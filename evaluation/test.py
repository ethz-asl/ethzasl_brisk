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

sm.setLoggingLevel(sm.LoggingLevel.Debug)

btree = sm.BoostPropertyTree()
btree.loadInfo('pipeline.info')

pipeline = acv.NCameraPipeline(sm.PropertyTree(btree, "UndistortingPipeline"))

bagin = rosbag.Bag('/mnt/data/Data-Server/Stefan/roundandroundandroundandround1_enhanced.bag')

gt_topic = '/vicon/brisk_roundabout/brisk_roundabout'
cam_topic = '/mv_cameras_manager/GX005924/image_raw'

gt_trans_list = []
gt_rot_list = []
for topic, msg, t in bagin.read_messages():
  print topic
  if topic == gt_topic:
    gt_trans = msg.transform.translation
    gt_rot = msg.transform.rotation
    gt_trans_list.append(gt_trans)
    gt_rot_list.append(gt_rot )

  if topic == cam_topic:
        img = Image.fromstring("L", [msg.width, msg.height], msg.data)
        stamp = acv.Time()
        stamp.fromNSec(t.to_nsec())
        mf = pipeline.addImage(stamp, 0, np.asanyarray(img))
        IPython.embed()
        for i in range(mf.numKeypoints()):
          kp = mf.keypoint(i)
          ray = kp.backProjection()
        

  
