import parseBag as pb
import getGroundTruthCorrespondencesBoost3 as ggt
import evaluation as ev
import dumpEvaluation as dev
import tools as tls

tag = 'camaware'
config = tag + '_config.info'
min_deg = 5
tag = tag + '_' + str(min_deg)
shelve = 'BRISK_frames_' + tag + '.shelve'
gtbin = 'BRISK_GT_' + tag + '.bin'

#pb.process(shelve, config)
#tls.extractFrameIndices(shelve, tag, min_deg)
#tls.buildEssentialMatrices(shelve, tag)
#ggt.process(shelve, gtbin, tag)
#ev.process(gtbin, shelve, tag)
#dev.process(tag) 

tag = 'camunaware'
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
dev.process(tag) 
