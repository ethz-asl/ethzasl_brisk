import rospy
import rosbag
import IPython
import Image as Img
import roslib
import numpy as np
import os
import Image

import sm
import aslam_cv as acv
import ShelveWrapper as sw
import aslam_simulation as sim

sm.setLoggingLevel(sm.LoggingLevel.Debug)

ts_corrector = sm.DoubleTimestampCorrector()

# Passing clearIntermediateResults will delete the file and start over
db = sw.ShelveDb('test.shelve', clear=True)

sm.setLoggingLevel(sm.LoggingLevel.Debug)

btree = sm.BoostPropertyTree()
btree.loadInfo('pipeline.info')

pipeline = acv.NCameraPipeline(sm.PropertyTree(btree, "UndistortingPipeline"))

bagin = rosbag.Bag('/mnt/data/Data-Server/Stefan/roundandroundandroundandround1_enhanced.bag')

gt_topic = '/vicon/brisk_roundabout/brisk_roundabout'
cam_topic = '/mv_cameras_manager/GX005924/image_raw'

traj = sim.DiscreteTrajectory()

gt_trans_list = []
gt_rot_list = []
index = 0

t_offset_vicon_cam = -54295835

for topic, msg, t in bagin.read_messages(topics=[gt_topic]):
  t_cor = ts_corrector.correctTimestamp(index, long(t.to_nsec()))
  gt_trans = msg.transform.translation
  gt_rot = msg.transform.rotation
  tr = np.array([gt_trans.x, gt_trans.y, gt_trans.z])
  q = np.array([gt_rot.x, gt_rot.y, gt_rot.z, gt_rot.w])
  T = sm.Transformation(q,tr)
  traj.addPose(long(t_cor), T)
  index += 1

tmin = traj.minTime()
tmax = traj.maxTime()

index = 0
for topic, msg, t in bagin.read_messages(topics=[cam_topic]):
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

  if True:
    mf = pipeline.addImage(stamp, 0, np.asanyarray(img))
    mf.setId(index)
    T_gt = traj.T(tnsec_gt)
    db[index] = (mf, T_gt)
    print 'stored frame ', index

  index += 1

db.sync()

  
