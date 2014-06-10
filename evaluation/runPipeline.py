import parseBag as pb
import getGroundTruthCorrespondencesBoost3 as ggt
import evaluation as ev
import plotEvaluation as pev
import dumpEvaluation as dev
import createVideo as hollywood
import tools as tls

reload(hollywood)

camaware_uni = False
camunaware_uni = False
camaware_nonuni = False
camunaware_nonuni = False
camaware_uni_video = True
camunaware_uni_video = True
camaware_nonuni_video = False
camunaware_nonuni_video = False

if camaware_uni:
  tag = 'camaware_uni'
  config = tag + '_config.info'
  min_deg = 5
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_' + tag + '.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'

  pb.process(shelve, config)
  tls.extractFrameIndices(shelve, tag, min_deg)
  tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  ev.process(gtbin, shelve, tag)
  pev.process(tag)
  dev.process(tag) 

if camaware_uni_video:
  tag = 'camaware_uni'
  config = tag + '_config.info'
  min_deg = 1
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_camaware_uni_5.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'

  #pb.process(shelve, config)
  tls.extractFrameIndices(shelve, tag, min_deg)
  #tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  hollywood.process(gtbin, shelve, tag)

if camunaware_uni:
  tag = 'camunaware_uni'
  config = tag + '_config.info'
  min_deg = 5
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_' + tag + '.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'

  pb.process(shelve, config)
  tls.extractFrameIndices(shelve, tag, min_deg)
  tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  ev.process(gtbin, shelve, tag)
  pev.process(tag)
  dev.process(tag) 

if camaware_uni_video:
  tag = 'camunaware_uni'
  config = tag + '_config.info'
  min_deg = 1
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_camunaware_uni_5.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'

  #pb.process(shelve, config)
  tls.extractFrameIndices(shelve, tag, min_deg)
  #tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  hollywood.process(gtbin, shelve, tag)

if camaware_nonuni:
  tag = 'camaware_nonuni'
  config = tag + '_config.info'
  min_deg = 5
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_' + tag + '.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'

  pb.process(shelve, config)
  tls.extractFrameIndices(shelve, tag, min_deg)
  tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  ev.process(gtbin, shelve, tag)
  pev.process(tag)
  dev.process(tag) 

if camunaware_nonuni:
  tag = 'camunaware_nonuni'
  config = tag + '_config.info'
  min_deg = 5
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_' + tag + '.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'

  pb.process(shelve, config)
  tls.extractFrameIndices(shelve, tag, min_deg)
  tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  ev.process(gtbin, shelve, tag)
  pev.process(tag)
  dev.process(tag) 
