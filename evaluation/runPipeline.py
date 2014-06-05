import parseBag as pb
import getGroundTruthCorrespondencesBoost3 as ggt
import evaluation as ev
import dumpEvaluation as dev

tag = 'camaware'
shelve = 'BRISK_frames_' + tag + '.shelve'
gtbin = 'BRISK_GT_' + tag + '.bin'

pb.process(shelve)
ggt.process(shelve, gtbin, tag)
ev.process(shelve, gtbin, tag)
dev.process(tag) 
