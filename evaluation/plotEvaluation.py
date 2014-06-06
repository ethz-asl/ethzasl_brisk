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

from matplotlib.pyplot import *

from collections import defaultdict

def main():
  prec_rec = pickle.load(open('BRISK_evaluation_camaware_precision_recall.bin'))
  repeat = pickle.load(open('BRISK_evaluation_camaware_repeatability.bin'))

  angles = defaultdict(list)

  for descriptorThreshold, angle, precision, recall in prec_rec:
    #if angle < 15.0:
    #  angles[10].append((descriptorThreshold, precision, recall))
    #else:
    #if angle < 0.0:
    #  angle = angle + 360.0
    angle = abs(angle)
    for a in range(0, 180, 10):
      if angle >= (a - 5.0) and angle < (a + 5.0):
        angles[a].append((descriptorThreshold, precision, recall))


  
  for a in range(0, 180, 10):

    f = figure(figsize=(20,12))
    for descriptorThreshold, precision, recall in angles[a]:
      c = (1.0 - descriptorThreshold/190.0, descriptorThreshold/190.0, descriptorThreshold/190.0)
      scatter([1.0 - precision], recall, color=c)


    xlabel('1 - Precision []')
    ylabel('Recall []')
    title('Precision vs. Recall - Angle: ' + str(a) + 'degs')
    grid(True)
    show()
    f.savefig('BRISK_prec_recall_' + str(a) + '_camaware.png')

  #f = figure(figsize=(20,12))
  #plot(angles, recalls, '.', color='g')
  #xlabel('yaw angle wrt. reference frame [deg]')
  #ylabel('recall []')
  #grid(True)
  #show()
  #f.savefig('BRISK_recall_camaware.png')###

  #f = figure(figsize=(20,12))
  #plot(angles, repeatabilities, '.', color='r')
  #xlabel('yaw angle wrt. reference frame [deg]')
  #ylabel('repeatability []')
  #grid(True)
  #show()
  #f.savefig('BRISK_repeatability_camaware.png')

if __name__ == '__main__':
    main()
