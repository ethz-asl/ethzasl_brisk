import rospy
import rosbag
import IPython
import Image as Img
import roslib
import numpy as np
import os
import Image
import time

import sm
import aslam_cv as acv
import ShelveWrapper as sw
import aslam_vcharge as vc

import sys

from matplotlib.pyplot import *

# dot product threshold for matches
threshold = 0.9995

#T_rig_camera = sm.Transformation(np.array([[ 0.02514192,  0.01364611,  0.99959075,  0.04200892],[-0.99959381, -0.01307991,  0.02532056,  0.00606903],[ 0.01342009, -0.99982133,  0.01331171, -0.01774598],[0,0,0,1]]))


# takes two multiframes and the ground truth transformation between them and returns a tuple
# with a list of all the matches and a dictionary with all the matches for a specific keypoint
def getGroundTruthCorrespondences(mf1, mf2, T_mf1_mf2, plot=True):
  n1 = mf1.numKeypoints()
  n2 = mf2.numKeypoints()
  candidates = {}
  matches = []
 
  for i in range(n1):
    kid1 = mf1.keypointIdentifier(i)
    kp1 = mf1.keypoint(kid1)
    ray1_c1 = kp1.backProjection()
    ray1_rig1 = np.dot(mf1.T_v_c(0).C(), ray1_c1)
    for j in range(n2):
      kid2 = mf2.keypointIdentifier(j)
      kp2 = mf2.keypoint(kid2)
      ray2_c2 = kp2.backProjection()
      ray2_rig2 = np.dot(mf2.T_v_c(0).C(), ray2_c2)
      ray2_rig1 = np.dot(T_mf1_mf2.C(), ray2_rig2)
      dt = ray1_rig1.dot(ray2_rig1)
      if dt > threshold:
        matches.append(acv.KeypointIdentifierMatch(kid1, kid2, dt))
        if candidates.has_key(i):
          candidates[i].append(j)
        else:
          candidates[i] = [j]

  if plot:
    clf()
    vc.util.plot.plotTwoMultiFrames(mf1, mf2)
    vc.util.plot.plotMultiFrameMatches(mf1, mf2, matches)
    show()

  return (matches, candidates)

def main():
  s = sw.ShelveDb('test.shelve')
  keys = s.keys()
  candidates = {}
  reference = 1400
  (mf_ref, T_w_ref) = s[reference]
  T_ref_w = T_w_ref.inverse()
  end = 3700
  sys.stdout.write("Computing ground truth correspondences!\n")
  sys.stdout.flush()
  i = reference + 1
  while(True):
    if i in keys:
      (mf_i, T_w_rigi) = s[i]
      T_rigref_rigi = T_ref_w * T_w_rigi
      (matches, candies) = getGroundTruthCorrespondences(mf_ref, mf_i, T_rigref_rigi, plot=False)
      candidates[i] = candies
      sys.stdout.write("\r")
      sys.stdout.write("% 2d of % 2d" % (i, end))
      sys.stdout.flush()
      i += 1
    else:
      break
    
  #kp1 = mf1.keypoint(71)
  #kp2 = mf2.keypoint(232)
  #ray1_c1 = kp1.backProjection()
  #ray1_rig1 = np.dot(mf1.T_v_c(0).C(), ray1_c1)
  #ray2_c2 = kp2.backProjection()
  #ray2_rig2 = np.dot(mf2.T_v_c(0).C(), ray2_c2)
  #ray2_rig1 = np.dot(T_rig1_rig2.C(), ray2_rig2)
  #T_w_mf1 = T_w_rig1 * mf1.T_v_c(0)
  #T_w_mf2 = T_w_rig2 * mf2.T_v_c(0)
  IPython.embed() 
  

if __name__ == '__main__':
    main()
