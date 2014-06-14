import parseBag as pb
import getGroundTruthCorrespondencesBoost3 as ggt
import evaluation as ev
import plotEvaluation as pev
import dumpEvaluation as dev
import createVideo as hollywood
import tools as tls

reload(hollywood)

camaware_uni = False
camunaware_uni = True
camaware_nonuni = True
camunaware_nonuni = True
camaware_uni_video = True
camunaware_uni_video = True
camaware_nonuni_video = True
camunaware_nonuni_video = True

if camaware_uni:
  tag = 'camaware_uni_ed'
  config = tag + '_config.info'
  min_deg = 5
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_' + tag + '.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'
  setExtractionDirection = True

  pb.process(shelve, config, setExtractionDirection)
  tls.extractFrameIndices(shelve, tag, min_deg)
  tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  ev.process(gtbin, shelve, tag)
  pev.process(tag)
  dev.process(tag) 

if camaware_uni_video:
  tag = 'camaware_uni_ed'
  config = tag + '_config.info'
  min_deg = 1
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_' + tag + '.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'
  setExtractionDirection = True

  pb.process(shelve, config, setExtractionDirection)
  tls.extractFrameIndices(shelve, tag, min_deg)
  tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  hollywood.process(gtbin, shelve, tag)

if camunaware_uni:
  tag = 'camunaware_uni_nonri'
  config = tag + '_config.info'
  min_deg = 5
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_' + tag + '.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'
  setExtractionDirection = False

  pb.process(shelve, config, setExtractionDirection)
  tls.extractFrameIndices(shelve, tag, min_deg)
  tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  ev.process(gtbin, shelve, tag)
  pev.process(tag)
  dev.process(tag) 

if camunaware_uni_video:
  tag = 'camunaware_uni_nonri'
  config = tag + '_config.info'
  min_deg = 1
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_' + tag + '.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'
  setExtractionDirection = Fals

  pb.process(shelve, config, setExtractionDirection)
  tls.extractFrameIndices(shelve, tag, min_deg)
  tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  hollywood.process(gtbin, shelve, tag)

if camaware_nonuni:
  tag = 'camaware_nonuni_ed'
  config = tag + '_config.info'
  min_deg = 5
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_' + tag + '.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'
  setExtractionDirection = True

  pb.process(shelve, config, setExtractionDirection)
  tls.extractFrameIndices(shelve, tag, min_deg)
  tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  ev.process(gtbin, shelve, tag)
  pev.process(tag)
  dev.process(tag) 

if camunaware_nonuni:
  tag = 'camunaware_nonuni_nonri'
  config = tag + '_config.info'
  min_deg = 5
  tag = tag + '_' + str(min_deg)
  shelve = 'BRISK_frames_' + tag + '.shelve'
  gtbin = 'BRISK_GT_' + tag + '.bin'
  setExtractionDirection = False

  pb.process(shelve, config, setExtractionDirection)
  tls.extractFrameIndices(shelve, tag, min_deg)
  tls.buildEssentialMatrices(shelve, tag)
  ggt.process(shelve, gtbin, tag)
  ev.process(gtbin, shelve, tag)
  pev.process(tag)
  dev.process(tag) 
