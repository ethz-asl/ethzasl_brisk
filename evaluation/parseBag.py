import rospy
import rosbag
import IPython
import Image as Img

import numpy as np
import os
import Image

import sm
import aslam_cv as acv
import ShelveWrapper as sw
import aslam_simulation as sim
import argparse

import aslam_vcharge as vc

def correctRosTimestamps(timestamps):
    corrector = sm.DoubleTimestampCorrector()
    correctedTimestamps = []
    for i in range(0,len(timestamps)):
        t = rospy.Time(corrector.correctTimestamp(i,timestamps[i].to_sec()))
        correctedTimestamps.append(t)
    return correctedTimestamps
    

def viconBagToTranslationAndRotation( bag, topicName):
    """Take a ros bag with vicon data and export lists with the rotation
    matrices and translation vectors.
    
    The translations are *not* scaled.
    
    Args:
        bagFile (string): the fullpath to the bag file to import
    
    Returns:
        timestamps (list(float)): ros timestamps at which the vicon messages were published
        rotations (list(np.array)): orientation of structure in the vicon reference frame
        translations (list(np.array)): position of the structure in the vicon reference frame
        bagtimestamps (list(float)): timestamps that the data was logged in the bag
    """
    timestamps = []
    rotations = []
    translations = []
    bagtimestamps = []
    
    if not type(bag) == rosbag.Bag:
        print "Opening Bag files: " + bagFile + "..."
        bag = rosbag.Bag(bag, 'r')

    for topic, msg, t in bag.read_messages(topics=topicName):    # read topics
        bagtimestamps.append(t)
        timestamps.append(msg.header.stamp)
        r = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
        rotations.append( sm.quat2r(np.array(r)).T )    # transpose to get correct toation representation!
        translations.append( [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        
    # NOTE: This assumes no messages are missed.
    correctedBagTimestamps = correctRosTimestamps(bagtimestamps)        
    return timestamps, rotations, translations, bagtimestamps, correctedBagTimestamps


def process(outputfile, config):
  sm.setLoggingLevel(sm.LoggingLevel.Debug)

  ts_corrector = sm.DoubleTimestampCorrector()

  # Passing clearIntermediateResults will delete the file and start over
  db = sw.ShelveDb(outputfile, clear=True)

  btree = sm.BoostPropertyTree()
  btree.loadInfo(config)

  pipeline = acv.NCameraPipeline(sm.PropertyTree(btree, "UndistortingPipeline"))

  bagin = rosbag.Bag('/mnt/data/Bags/Stefan/roundandroundandroundandround1_enhanced.bag')

  gt_topic = '/vicon/brisk_roundabout/brisk_roundabout'
  cam_topic = '/mv_cameras_manager/GX005924/image_raw'

  # parsing vicon
  print 'parsing vicon data...'
  timestamps, rotations, translations, bagtimestamps, correctedBagTimestamps = viconBagToTranslationAndRotation(bagin, gt_topic)
  print 'done'

  traj = sim.DiscreteTrajectory()

  gt_trans_list = []
  gt_rot_list = []
  index = 0

  t_offset_vicon_cam = -54295835

  for ct, rot, trans in zip(correctedBagTimestamps, rotations, translations):
    ti = long(ct.to_nsec())
    T = np.identity(4)
    T[0:3,0:3] = rot
    T[0:3,3] = trans
    T = sm.Transformation(T)
    traj.addPose(ti, T)

  #for topic, msg, t in bagin.read_messages(topics=[gt_topic]):
  #  t_cor = ts_corrector.correctTimestamp(index, long(t.to_nsec()))
  #  gt_trans = msg.transform.translation
  #  gt_rot = msg.transform.rotation
  #  tr = np.array([gt_trans.x, gt_trans.y, gt_trans.z])
  #  q = np.array([gt_rot.x, gt_rot.y, gt_rot.z, gt_rot.w])
  #  T = sm.Transformation(q,tr)
  #  traj.addPose(long(t_cor), T)
  #  index += 1

  tmin = traj.minTime()
  tmax = traj.maxTime()

  index = 0
  for topic, msg, t in bagin.read_messages(topics=[cam_topic]):
    #if limit > 0 and index > limit:
    #  break

    tnsec = t.to_nsec()
    tnsec_gt = long(tnsec + t_offset_vicon_cam)
    if (tnsec_gt < tmin) or (tnsec_gt > tmax):
      continue

    img = Image.fromstring("L", [msg.width, msg.height], msg.data)
    stamp = acv.Time()
    stamp.fromNSec(tnsec)
    #print 'adding frame ', index
    #if index == 785:
    #  IPython.embed()
    #if index < 1400:
    #  index += 1
    #  continue

    if True:
      mf = pipeline.addImage(stamp, 0, np.asanyarray(img))
      #if index == 0:
      #IPython.embed()

      mf.setId(index)
      T_w_rig_gt= traj.T(tnsec_gt)
      db[index] = (mf, T_w_rig_gt)
      print 'stored frame ', index

    index += 1

  db.sync()

def main():
    parser = argparse.ArgumentParser(description="parse the BRISK evaluation data")

    #parser.add_argument("inputFile", type=str, help="")
    parser.add_argument("outputshelve", type=str, help="")

    args = parser.parse_args()

    #inputfile = str(args.inputFile)
    outputfile = str(args.outputFile)

    process(outputfile)

if __name__ == '__main__':
    main()
