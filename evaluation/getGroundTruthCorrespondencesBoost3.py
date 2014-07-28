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
import math
import sys
import aslam_vcharge as vc

import pickle
from collections import defaultdict

from matplotlib.pyplot import *

# dot product threshold for matches
threshold = 0.9995
focalLength = 914.0
scale_range = 1.6
scale_range_inv = 1.0 / scale_range
viconUncertainty = 0.05
viconUncertainty2 = viconUncertainty * viconUncertainty
#T_rig_camera = sm.Transformation(np.array([[ 0.02514192,  0.01364611,  0.99959075,  0.04200892],[-0.99959381, -0.01307991,  0.02532056,  0.00606903],[ 0.01342009, -0.99982133,  0.01331171, -0.01774598],[0,0,0,1]]))


#essentialMatrices = {}

def boolMatrixToDictionary(G):
  candidates = defaultdict(list)
  it = np.nditer(G, flags=['multi_index'])
  while not it.finished:
    if it[0]:
      candidates[it.multi_index[0]].append(it.multi_index[1])
    it.iternext()
  
  return candidates

def boolMatrixToList(G, idA, idB):
  matches = []
  it = np.nditer(G, flags=['multi_index'])
  while not it.finished:
    if it[0]:
      matches.append(acv.KeypointIdentifierMatch(acv.KeypointIdentifier(idA,0,it.multi_index[0]), acv.KeypointIdentifier(idB, 0, it.multi_index[1]), 0.0))
    it.iternext()

  return matches
  
# takes two multiframes and the ground truth transformation between them and returns a tuple
# with a list of all the matches and a dictionary with all the matches for a specific keypoint
def getGroundTruthCorrespondences(M1_c1, S1, M2_c2, S2, b, n1, n2, T_c1_c2, mf1, mf2, plot=True):
  S1_full = 3.0 * np.tile(S1, (1, n2))  
  S2_full = 3.0 * np.tile(S2.transpose(), (n1, 1)) 
  
  viconUncertainty2M = np.ones((n1, n2)) * viconUncertainty2
  S_sum = np.sqrt(np.square((S1_full + S2_full) / focalLength) + viconUncertainty2M)

  #T_c1_c2 = mf1.T_v_c(0).inverse() * T_mf1_mf2 * mf2.T_v_c(0)

  M2_c1 = np.dot(T_c1_c2.C(), M2_c2.transpose())

  M = np.arccos(np.dot(M1_c1, M2_c1))
  S = np.dot(S1, (1.0 / S2.transpose()))

  S_t = (S > scale_range_inv) & (S < scale_range)
  H = M < S_sum
  G = S_t & H

  #candidates = boolMatrixToDictionary(G)
  candidates = defaultdict(list)
  it = np.nditer(G, flags=['multi_index'])
  while not it.finished:
    if it[0]:
      i = it.multi_index[0]
      j = it.multi_index[1]
      candidates[i].append((j, H[i][j]))
    it.iternext()
  
  if plot:
    matches = boolMatrixToList(G, mf1.id(), mf2.id())
    clf()
    vc.util.plot.plotTwoMultiFrames(mf1, mf2)
    vc.util.plot.plotMultiFrameMatches(mf1, mf2, matches)
    show()

  return candidates

def process(inputshelve, outputbin, tag):
  s = sw.ShelveDb(inputshelve)
  keys = s.keys()
  candidates = {}
  #startFrame = 1400
  #endFrame = 2700

  idxs = pickle.load(open('indices_' + tag + '.bin'))
  essentialMatrices = pickle.load(open('essentialMatrices_' + tag + '.bin'))
  
  for i in range(len(idxs)):
    a = idxs[i]
    mfA, T_w_a = s[a]
    n1 = mfA.numKeypoints()
    T_ca_w = mfA.T_v_c(0).inverse() * T_w_a.inverse()
    M1_c1, S1 = essentialMatrices[a]
    for j in range(i+1, len(idxs)):
      b = idxs[j]
      t0 = time.time()
      print '--------------- at frameA: ', a, ' frameB: ', b
      mfB, T_w_b = s[b]
      T_ca_cb =  T_ca_w * T_w_b * mfB.T_v_c(0)
      n2 = mfB.numKeypoints()
      M2_c2, S2 = essentialMatrices[b]
      candies = getGroundTruthCorrespondences(M1_c1, S1, M2_c2, S2, b, n1, n2, T_ca_cb, mfA, mfB, plot=False)
      candidates[(a,b)] = candies
      t1 = time.time()
      print 't: ', t1-t0

  pickle.dump(candidates, open(outputbin, 'w'))

def main():
    parser = argparse.ArgumentParser(description="compute the BRISK ground truth")

    parser.add_argument("inputshelve", type=str, help="")
    parser.add_argument("outputbin", type=str, help="")
    parser.add_argument("tag", type=str, help="")

    args = parser.parse_args()

    inputshelve = str(args.inputshelve)
    outputbin = str(args.outputbin)
    tag = str(args.tag)

    process(inputshelve, outputbin, tag)

if __name__ == '__main__':
    main()
