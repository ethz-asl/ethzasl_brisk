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

def process(tag):
  prec_rec = pickle.load(open('BRISK_evaluation_' + tag + '_precision_recall.bin'))
  repeat = pickle.load(open('BRISK_evaluation_' + tag + '_repeatability.bin'))
  sep = ','
  newline = '\n'

  f = open('BRISK_evaluation_' + tag + '_precision_recall.txt', 'w')
  f.write('angle [deg], descriptorThreshold [], precision [], recall []' + newline)
  for descriptorThreshold, angle, precision, recall in prec_rec:
    f.write(str(abs(angle)) + sep + str(descriptorThreshold) + sep + str(precision) + sep + str(recall) + newline)

  f.close()

  f = open('BRISK_evaluation_' + tag + '_repeatability.txt', 'w')
  f.write('angle [deg], repeatability []' + newline)
  for angle, repeatability in repeat:
    f.write(str(abs(angle)) + sep + str(repeatability) + newline)

  f.close()

def main():
    parser = argparse.ArgumentParser(description="compute the BRISK ground truth")

    parser.add_argument("tag", type=str, help="")

    args = parser.parse_args()

    tag = str(args.tag)

    process(tag)

if __name__ == '__main__':
    main()
