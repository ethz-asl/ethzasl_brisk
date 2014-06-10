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

import argparse

#import getGroundTruthCorrespondences as gGTC

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

def invertDictionary(d):
  d_inv = {}
  for key, value in d.iteritems():
    for v in value:
      if d_inv.has_key(v):
        d_inv[v].append(key)
      else:
        d_inv[v] = [key]

  return d_inv

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

def dictToList(d, idA, idB):
  matches = []
  for key, value in d.iteritems():
    for v in value:
      matches.append(acv.KeypointIdentifierMatch(acv.KeypointIdentifier(idA ,0 ,key), acv.KeypointIdentifier(idB, 0, v), 0.0))

  return matches

def forceOneToOneMatching(d):
  doto = {}
  for key, value in d.iteritems():
    m = 0.0
    j = 0
    for index, dist in value:
      if dist > m:
        j = index
        m = dist

    doto[key] = (j, m)

  #invert it
  dinv = {}
  for key, value in doto.iteritems():
    j = value[0]
    d = value[1]
    if dinv.has_key(j):
      ic, dc = dinv[j]
      if d > dc:
        #replace
        dinv[j] = (key, d)
    else:
      dinv[j] = (key, d)

  return dinv

  

def computeRepeatability(mfA, mfB, T_A_B, gt_correspondences):
  gt_correspondences_inv = invertDictionary(gt_correspondences)
  gt_onetoone = forceOneToOneMatching(gt_correspondences)
  numKeypointsA = mfA.numKeypoints()
  numKeypointsB = mfB.numKeypoints()
  numValidA = 0
  numValidB = 0
  numberOfCorrespondences = 0
  #numberOfCorrespondencesB = 0
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
      #if gt_onetoone.has_key(i):
      #  numberOfCorrespondencesA += 1    
    #else:
      #if gt_onetoone.has_key(i):
      #  print 'WARNING! keypint ', i, 'in the reference frame is considered outside the common field of view but has a GT correspondence.'
      #  #IPython.embed()

  #if len(gt_correspondences) != numberOfCorrespondences:
  #  print 'ERROR! mismtach between the number of correspondences and the number of correspondences ;-)'
  #  #IPython.embed()

  for i in range(numKeypointsB):
    ray_cb = mfB.keypoint(i).backProjection()
    # transform the ray to frame A
    ray_rigB = np.dot(mfB.T_v_c(0).C(), ray_cb)
    ray_rigA = np.dot(T_A_B.C(), ray_rigB)
    ray_ca = np.dot(mfA.T_v_c(0).inverse().C(), ray_rigA)
    visibleInA = geoA.isEuclideanVisible(ray_ca)

    if visibleInA:
      numValidB += 1
      if gt_onetoone.has_key(i):
        numberOfCorrespondences += 1
    #else:
    #  if gt_correspondences_inv.has_key(i):
    #    print 'WARNING! keypint ', i, 'in  frame B is considered outside the common field of view but has a GT correspondence.'

  #if len(gt_correspondences_inv) != numberOfCorrespondencesB:
  #  print 'ERROR! mismtach between the number of inv correspondences and the number of correspondences ;-)'
  #  #IPython.embed()

  minKeypoints = min(numValidA, numValidB)
  if minKeypoints == 0:
    return 0

  repeatability = float(numberOfCorrespondences) / float(minKeypoints)
  return repeatability

def evaluateMatches(matches, gt_correspondences, mfRef, mfB):
  numRightPositives = 0
  numFalsePositives = 0
  for m in matches:
    kiRef = m.getItem0().keypointIndex
    kiB = m.getItem1().keypointIndex
    if gt_correspondences.has_key(kiRef):
      gts = [gt for gt, d in gt_correspondences[kiRef] ]
      if kiB in gts:
        numRightPositives += 1
      else:
        numFalsePositives += 1
    else:
      numFalsePositives += 1

  return (numRightPositives, numFalsePositives)

def splitMatches(matches, gt_dict):
  goodMatches = []
  badMatches = []
  for m in matches:
    ka = m.getItem0().keypointIndex
    kb = m.getItem1().keypointIndex
    good = False
    if gt_dict.has_key(ka):
      for v, d in gt_dict[ka]:
        if v == kb:
          good = True
          break

    if good:
      goodMatches.append(m)
    else:
      badMatches.append(m)

  return (goodMatches, badMatches)

def process(gtinputbin, inputshelve, tag):
  print 'creating video'
  gt_data = pickle.load(open(gtinputbin))
  s = sw.ShelveDb(inputshelve)
  idxs = pickle.load(open('indices_' + tag + '.bin'))

  btree = sm.BoostPropertyTree()
  btree.loadInfo('pipeline.info')

  matcher = acv.DescriptorMatcher(sm.PropertyTree(btree, "Matcher"))

  nindices = len(idxs)

  i = 0
  endFrame = 120

  data = []
  repeatabilities = []


  t0 = time.time()
  a = idxs[i]
  mfA, T_w_a = s[a]
  for j in range(i+1, endFrame):
    t1 = time.time()
    b = idxs[j]
    print '----------- frame A: ', a, ' Frame B: ', b
    mfB, T_w_b = s[b]
    T_a_b = T_w_a.inverse() * T_w_b
    angle = sm.rad2deg(sm.R2rph(T_a_b.C())[2])

    #(gt_matches, gt_dict) = gGTC.getGroundTruthCorrespondences(mfA, mfB, T_a_b)
    gt_dict = gt_data[(a,b)]

    descriptorThreshold = 55
    t4 = time.time()
    #print 'oooooooooooooooooooooooooooooooo threshold: ', descriptorThreshold
    matcher.setDescriptorDistanceThreshold(descriptorThreshold)
    
    matches = matcher.match2D2D(mfA, mfB)
    goodMatches, badMatches = splitMatches(matches, gt_dict)
    if len(matches) > 0:
      f = figure(figsize=(12, 18))
      vc.util.plot.plotTwoMultiFrames(mfA, mfB, keypointColor='y')
      vc.util.plot.plotMultiFrameMatches(mfA, mfB, goodMatches  )
      vc.util.plot.plotMultiFrameMatches(mfA, mfB, badMatches, lineColor='r')
      axis('off')
      #f.title(tag + ' - descriptor matching threshold = 55')
      tight_layout()
      nbr = str(j).zfill(6)
      f.savefig('video/' + tag + '/frame' + nbr + '.png')
      #f.close()


def main():
    parser = argparse.ArgumentParser(description="compute the BRISK ground truth")

    parser.add_argument("gtinputbin", type=str, help="")
    parser.add_argument("inputshelve", type=str, help="")
    parser.add_argument("tag", type=str, help="")
    #parser.add_argument("min_deg", type=str, help="")

    args = parser.parse_args()

    gtinputbin = str(args.gtinputbin)
    inputshelve = str(args.inputshelve)
    tag = str(args.tag)
    #min_deg = str(args.min_deg)

    process(gtinputbin, inputshelve, tag)

if __name__ == '__main__':
    main()
