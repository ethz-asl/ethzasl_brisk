import parseBag as pb
import getGroundTruthCorrespondencesBoost3 as ggt
import evaluation as ev
import plotEvaluation as pev
import dumpEvaluation as dev
import tools as tls
import pickle
import ShelveWrapper as sw
import aslam_vcharge as vc
import matplotlib.pyplot as plt

tag = 'camaware_uni'
config = tag + '_config.info'
min_deg = 5
tag = tag + '_' + str(min_deg)
shelve = 'BRISK_frames_' + tag + '.shelve'
gtbin = 'BRISK_GT_' + tag + '.bin'

s = sw.ShelveDb(shelve)
essentialMatrices = pickle.load(open('essentialMatrices_' + tag + '.bin'))

a = 1800
b = 1900
mfA, T_w_a = s[a]
mfB, T_w_b = s[b]
n1 = mfA.numKeypoints()
n2 = mfB.numKeypoints()
T_ca_w = mfA.T_v_c(0).inverse() * T_w_a.inverse()
M1_c1, S1 = essentialMatrices[a]
T_ca_cb =  T_ca_w * T_w_b * mfB.T_v_c(0)
M2_c2, S2 = essentialMatrices[b]
candies = ggt.getGroundTruthCorrespondences(M1_c1, S1, M2_c2, S2, b, n1, n2, T_ca_cb, mfA, mfB, plot=True)

tag = 'camunaware_uni'
config = tag + '_config.info'
min_deg = 5
tag = tag + '_' + str(min_deg)
shelve = 'BRISK_frames_' + tag + '.shelve'
gtbin = 'BRISK_GT_' + tag + '.bin'

s_u = sw.ShelveDb(shelve)

mfu, T = s_u[1800]

plt.figure()
vc.util.plot.plotTwoMultiFrames(mfA, mfu)
plt.show()
