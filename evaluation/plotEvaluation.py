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

def main():
  angles, precisions, recalls = pickle.load(open('BRISK_evaluation.bin'))
  f = figure(figsize=(20,12))
  plot(angles, precisions, '.', color='b')
  xlabel('yaw angle wrt. reference frame [deg]')
  ylabel('precision []')
  grid(True)
  show()
  f.savefig('BRISK_precision_camaware.png')
  f = figure(figsize=(20,12))
  plot(angles, recalls, '.', color='g')
  xlabel('yaw angle wrt. reference frame [deg]')
  ylabel('recall []')
  grid(True)
  show()
  f.savefig('BRISK_recall_camaware.png')


if __name__ == '__main__':
    main()
