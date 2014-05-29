import rospy
import rosbag
import IPython
import Image as Img
import roslib
import numpy as np
import os
import Image
import time
import pickle

import sm
import aslam_cv as acv
import ShelveWrapper as sw
import aslam_vcharge as vc

from matplotlib.pyplot import *

def getKeypointFurthestLeftAndRight(mf):
  xmin = 1e12
  xmax = 0
  kpleft = 0
  kpright = 0
  n = mf.numKeypoints()
  for i in range(n):
    kp = mf.keypoint(i)
    x = kp.y()[0]
    if x < xmin:
      xmin = x
      kpleft = i
    if x > xmax:
      xmax = x
      kpright = i

  return (kpleft, kpright)

def badShitThatWasTotallyUseless():
  angle = sm.rad2deg(sm.R2rph(T_A_B.C())[2])

  lA, rA = getKeypointFurthestLeftAndRight(mfA)
  lB, rB = getKeypointFurthestLeftAndRight(mfB)
  #rayLa_ca = mfA.keypoint(lA).backProjection()
  #rayRa_ca = mfA.keypoint(rA).backProjection()
  #rayLb_cb = mfB.keypoint(lB).backProjection()
  #rayRb_cb = mfB.keypoint(rB).backProjection()
  rayLa_ca = np.array([-1.0, 0.0, 0.0])
  rayRa_ca = np.array([1.0, 0.0, 0.0])
  rayLb_cb = np.array([-1.0, 0.0, 0.0])
  rayRb_cb = np.array([1.0, 0.0, 0.0])

  rayLa_rigA = np.dot(mfA.T_v_c(0).C(), rayLa_ca)
  rayRa_rigA = np.dot(mfA.T_v_c(0).C(), rayRa_ca)
  rayLb_rigB = np.dot(mfB.T_v_c(0).C(), rayLb_cb)
  rayRb_rigB = np.dot(mfB.T_v_c(0).C(), rayRb_cb)
  rayLa_rigB = np.dot(T_A_B.inverse().C(), rayLa_rigA)
  rayRa_rigB = np.dot(T_A_B.inverse().C(), rayRa_rigA)
  rayLb_rigA = np.dot(T_A_B.C(), rayLb_rigB)
  rayRb_rigA = np.dot(T_A_B.C(), rayRb_rigB)
  rayLa_cb = np.dot(mfB.T_v_c(0).inverse().C(), rayLa_rigB)
  rayRa_cb = np.dot(mfB.T_v_c(0).inverse().C(), rayRa_rigB)
  rayLb_ca = np.dot(mfA.T_v_c(0).inverse().C(), rayLb_rigA)
  rayRb_ca = np.dot(mfA.T_v_c(0).inverse().C(), rayRb_rigA)
  # project all rays onto the x/z plane (i.e. the horizontal image plane through the image center)
  rayLa_ca_p = np.array([rayLa_ca[0], rayLa_ca[2]])
  rayRa_ca_p = np.array([rayRa_ca[0], rayRa_ca[2]])
  rayLa_cb_p = np.array([rayLa_cb[0], rayLa_cb[2]])
  rayRa_cb_p = np.array([rayRa_cb[0], rayRa_cb[2]])
  rayLb_ca_p = np.array([rayLb_ca[0], rayLb_ca[2]])
  rayRb_ca_p = np.array([rayRb_ca[0], rayRb_ca[2]])
  rayLb_cb_p = np.array([rayLb_cb[0], rayLb_cb[2]])
  rayRb_cb_p = np.array([rayRb_cb[0], rayRb_cb[2]])
  # renormalize and extract x value 
  rayLa_ca_x = (rayLa_ca_p / np.linalg.norm(rayLa_ca_p))[0]
  rayRa_ca_x = (rayRa_ca_p / np.linalg.norm(rayRa_ca_p))[0]
  rayLa_cb_x = (rayLa_cb_p / np.linalg.norm(rayLa_cb_p))[0]
  rayRa_cb_x = (rayRa_cb_p / np.linalg.norm(rayRa_cb_p))[0]
  rayLb_ca_x = (rayLb_ca_p / np.linalg.norm(rayLb_ca_p))[0]
  rayRb_ca_x = (rayRb_ca_p / np.linalg.norm(rayRb_ca_p))[0]
  rayLb_cb_x = (rayLb_cb_p / np.linalg.norm(rayLb_cb_p))[0]
  rayRb_cb_x = (rayRb_cb_p / np.linalg.norm(rayRb_cb_p))[0]


def computeRepeatability(mfA, mfB, T_A_B, gt_correspondences):
  numKeypointsA = mfA.numKeypoints()
  numKeypointsB = mfB.numKeypoints()
  numValidA = 0
  numValidB = 0
  numberOfCorrespondences = 0
  geoA = mfA.getFrame(0).geometry()
  geoB = mfB.getFrame(0).geometry()
  for i in range(numKeypointsA):
    ray_ca = mfA.keypoint(i).backProjection()
    # transform the ray to frame B
    ray_rigA = np.dot(mfA.T_v_c(0).C(), ray_ca)
    ray_rigB = np.dot(T_A_B.inverse().C(), ray_rigA)
    ray_cb = np.dot(mfB.T_v_c(0).inverse().C(), ray_rigB)
    visibleInB = geoB.isEuclideanVisible(ray_cb)

    if visibleInB:
      numValidA += 1
      if gt_correspondences.has_key(i):
        numberOfCorrespondences += 1    
    else:
      if gt_correspondences.has_key(i):
        print 'WARNING! keypint ', i, 'in the reference frame is considered outside the common field of view but has a GT correspondence.'
        IPython.embed()

  if len(gt_correspondences) != numberOfCorrespondences:
    print 'ERROR! mismtach between the number of correspondences and the number of correspondences ;-)'
    IPython.embed()

  for i in range(numKeypointsB):
    ray_cb = mfB.keypoint(i).backProjection()
    # transform the ray to frame A
    ray_rigB = np.dot(mfB.T_v_c(0).C(), ray_cb)
    ray_rigA = np.dot(T_A_B.C(), ray_rigB)
    ray_ca = np.dot(mfA.T_v_c(0).inverse().C(), ray_rigA)
    visibleInA = geoA.isEuclideanVisible(ray_ca)

    if visibleInA:
      numValidB += 1

  minKeypoints = min(numValidA, numValidB)
  if minKeypoints == 0:
    return 0
  else: 
    repeatability = float(numberOfCorrespondences) / float(minKeypoints)
    if repeatability > 1.0:
      IPython.embed()
    return repeatability

def evaluateMatches(matches, gt_correspondences, mfRef, mfB):
  numRightPositives = 0
  numFalsePositives = 0
  for m in matches:
    kiRef = m.getItem0().keypointIndex
    kiB = m.getItem1().keypointIndex
    if gt_correspondences.has_key(kiRef):
      gts = gt_correspondences[kiRef]
      if kiB in gts:
        numRightPositives += 1
      else:
        numFalsePositives += 1
    else:
      numFalsePositives += 1

  return (numRightPositives, numFalsePositives)

def main():
  gt_data = pickle.load(open('BRISK_gt.bin'))
  s = sw.ShelveDb('BRISK_bag.shelve')

  btree = sm.BoostPropertyTree()
  btree.loadInfo('pipeline.info')

  matcher = acv.MultiFrameTracker(sm.PropertyTree(btree, "Matcher"))

  mfRef, T_w_ref = s[1400]
  
  #mfB, T_w_b = s[1670]
  #T_ref_b = T_w_ref.inverse() * T_w_b
  #bla(mfRef, mfB, T_ref_b, gt_data[1670])
  
  angles = []
  precisions = []
  recalls = []
  repeatabilities = []
  for i in range(1401, 3574):
    mfB, T_w_b = s[i]
    T_ref_b = T_w_ref.inverse() * T_w_b

    repeatability = computeRepeatability(mfRef, mfB, T_ref_b, gt_data[i])
    repeatabilities.append(repeatability)

    a_deg = sm.rad2deg(sm.R2rph(T_ref_b.C())[2])
    angles.append(a_deg)

    matches = matcher.match2D2D(mfRef, mfB)
    numRP, numFP = evaluateMatches(matches, gt_data[i], mfRef, mfB)
    precision = 1.0 - (float(numFP) / float((numFP + numRP)))
    numGtCorrespondences = len(gt_data[i])
    if numGtCorrespondences > 0:
      recall = float(numRP) / float(numGtCorrespondences)
    else:
      recall = 0.0

    precisions.append(precision)
    recalls.append( recall)   

  pickle.dump((angles, precisions, recalls, repeatabilities), open('BRISK_evaluation.bin', 'w'))

if __name__ == '__main__':
    main()
