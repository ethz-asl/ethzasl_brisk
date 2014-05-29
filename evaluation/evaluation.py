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
  
  angles = []
  precisions = []
  recalls = []
  for i in range(1401, 3574):
    mfB, T_w_b = s[i]
    T_ref_b = T_w_ref.inverse() * T_w_b
    a_deg = sm.rad2deg(sm.R2rph(T_ref_b.C())[2])
    angles.append(a_deg)

    matches = matcher.match2D2D(mfRef, mfB)
    numRP, numFP = evaluateMatches(matches, gt_data[i], mfRef, mfB)
    precision = 1.0 - (float(numFP) / float((numFP + numRP)))
    recall = (float(numRP) / float(len(gt_data[1600])))

    precisions.append(precision)
    recalls.append( recall)   

  pickle.dump((angles, precisions, recalls), open('BRISK_evaluation.bin', 'w'))

if __name__ == '__main__':
    main()
