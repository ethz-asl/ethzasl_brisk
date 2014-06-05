import rospy
import sm
import cv
import cv_bridge
import cv2
import rosbag
import os
import numpy as np
import pylab as pl
import math
import bsplines
import time

np.set_printoptions(suppress=True)


def t2params( T ):
    p = np.zeros([6,1])
    p[0:3,0] = T[0:3,3]
    p[3:6,0] = sm.R2AxisAngle(T[0:3,0:3])
    return p
def t2err( T ):
    p = t2params(T)
    err = np.array([ np.linalg.norm(p[0:3]), np.linalg.norm(p[3:6])])
    return err

def buildT( C, t ):
    T = np.eye(4)
    T[0:3,0:3] = C
    T[0:3,3] = np.array(t).flatten()
    return T

class Vicon(object):
    pass
class Camera(object):
    pass
class Estimate(object):
    pass
class DesignVariables(object):
    pass
class Output(object):
    pass

def correctSketchyTimestamps(timestamps):
    """Correct timestamps that come at a nominally fixed rate but have missing messages"""
    # First build a double array.
    if type(timestamps[0]) == aslam.cv.Time or\
            type(timestamps[0]) == rosbag.bag.genpy.Time or\
            type(timestamps[0]) == rospy.Time:
        T = np.array([t.to_sec() for t in timestamps])
    else:
        T = np.array([t for t in timestamps])
    
    # Compute the median time between timestamps.
    # This is like the nominal 
    dT = T[1:] - T[:-1]
    med_dt = np.median(dT)
    print "Median dt: {0}".format(med_dt)
    
    # Now, try to figure out the sequence number
    tcorr = sm.DoubleTimestampCorrector()
    corrected = []
    seqs = []
    # The sequence number of the measurement
    seq = 0
    corrected.append(tcorr.correctTimestamp( seq, T[0] ))
    seqs.append(seq)
    for i in range(1,len(T)):
        dtk = dT[i-1]
        # This should work? if the timestamps are
        # More jittery than this, we have little hope.
        # \todo this won't deal with a delay followed by 
        # bunched up measurements. For that, we will need
        # something fancier.
        dseq = round( dtk / med_dt )
        if dseq == 0:
            raise RuntimeError("Found a step of zero!")
        seq = seq + dseq
        corrected.append(tcorr.correctTimestamp( seq, T[i] ))
        seqs.append(seq)

    corrected = np.array(corrected)
    # Now, just because we can, batch correct
    c2 = np.array([ tcorr.getLocalTime(seq) for seq in seqs])
    return (T,dT,med_dt,c2,seqs,tcorr)

def correctRosTimestamps(timestamps):
    corrector = sm.DoubleTimestampCorrector()
    correctedTimestamps = []
    for i in range(0,len(timestamps)):
        t = rospy.Time(corrector.correctTimestamp(i,timestamps[i].to_sec()))
        correctedTimestamps.append(t)
    return correctedTimestamps
    

def viconBagToTranslationAndRotation( bag, topicName):
    """Take a ros bag with vicon data and export lists with the rotation
    matrices and translation vectors.
    
    The translations are *not* scaled.
    
    Args:
        bagFile (string): the fullpath to the bag file to import
    
    Returns:
        timestamps (list(float)): ros timestamps at which the vicon messages were published
        rotations (list(np.array)): orientation of structure in the vicon reference frame
        translations (list(np.array)): position of the structure in the vicon reference frame
        bagtimestamps (list(float)): timestamps that the data was logged in the bag
    """
    timestamps = []
    rotations = []
    translations = []
    bagtimestamps = []
    
    if not type(bag) == rosbag.Bag:
        print "Opening Bag files: " + bagFile + "..."
        bag = rosbag.Bag(bag, 'r')

    for topic, msg, t in bag.read_messages(topics=topicName):    # read topics
        bagtimestamps.append(t)
        timestamps.append(msg.header.stamp)
        r = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
        rotations.append( sm.quat2r(np.array(r)).T )    # transpose to get correct toation representation!
        translations.append( [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        
    # NOTE: This assumes no messages are missed.
    correctedBagTimestamps = correctRosTimestamps(bagtimestamps)        
    return timestamps, rotations, translations, bagtimestamps, correctedBagTimestamps

def loadCheckerboardData(bag, topic, cameraModel, grid, observations, images):
    if not type(bag) == rosbag.Bag:
        bag = rosbag.Bag( bag, 'r' )
    
    print "initializing detector"
    detector = acc.cv.GridDetector(cameraModel.geometry, grid, True, False, False, True)

    CVB = cv_bridge.CvBridge()

    timestamps = []
    bagTimestamps = []
    snappyTimestamps = []
    success = []
    print "Extracting checkerboard corners"
    i = 0
    for topic, msg, t in bag.read_messages(topics=topic):    # read topics
        if msg._type == 'mv_cameras/ImageSnappyMsg':
            from snappy import uncompress
            image = np.reshape(uncompress(np.fromstring(msg.data, dtype='uint8')),(msg.height,msg.width), order="C")
            snappyTimestamps.append(msg.hwTimestamp)
        else:
            image = CVB.imgmsg_to_cv(msg)
        bagTimestamps.append(t)
        timestamps.append(msg.header.stamp)
        #observations.append
        stamp = acc.cv.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
        suc, observation = detector.findTarget(stamp, np.array(image))
        im = observation.getImage()
        observation.clearImage()
        success.append(suc)
        observations[i] = observation
        images[i] = im
        i = i + 1
        if i % 100 == 0:
            print "processed {0} images".format(i)
            observations.sync()
    correctedBagTimestamps = correctRosTimestamps(bagTimestamps)
    observations.sync()
    
    return timestamps, success, observations, bagTimestamps, correctedBagTimestamps, snappyTimestamps, images


class CameraToViconCalibrator(object):
    def __init__(self, tag, bagFileName, viconTopic, cameras, viconUseBagTime, cameraUseBagTime, grid, correctViconTime=True, correctCameraTime=True, clearIntermediateResults=False):
        # Create a database to cache intermediate results.
        self.dbName = '{0}.db'.format(tag)
        print "Opening database. Clear: {0}".format(clearIntermediateResults)
        self.db = sww.ShelveDb(self.dbName,clearIntermediateResults)
        print "  Database keys: {0}".format(self.db.keys())
        self.bagFileName = bagFileName
        # First, get all the vicon data.
        print "Using vicon topic: {0}".format(viconTopic)
        self.processViconData(viconTopic,viconUseBagTime,correctTime=correctViconTime)
        print "Using camera topics: {0}".format([cam.topic for cam in cameras])
        self.processImageData(cameras, grid, cameraUseBagTime, correctTime=correctCameraTime)
        self.db.sync()
    def __del__(self):
        self.db.sync()
    def processViconData(self, viconTopic, useBagTime, correctTime=True):
        if self.db.has_key('vicon'):
            print "Loading previously processed vicon data"
            self.vicon = self.db['vicon']
        else:
            print "Opening bag file: {0}".format(self.bagFileName)
            bag = rosbag.Bag(self.bagFileName, 'r')

            print "Parsing vicon data"
            timestamps, rotations, translations, bagtimestamps, correctedBagTimestamps = viconBagToTranslationAndRotation(bag, viconTopic);
            vicon = Vicon()
            T_vicon_rig = [ sm.Transformation(buildT( rotations[i], translations[i])) for i in range(0,len(rotations)) ]
            vicon.T_vicon_rig = T_vicon_rig
            vicon.C_vicon_rig = rotations
            vicon.t_vicon_rig = translations
            if useBagTime:
                vicon.timestamps = correctedBagTimestamps
            else:
                vicon.timestamps = timestamps
                
            # Postprocess time
            if correctTime:
                T,dT,med_dt,corrected,seqs,tcorr = correctSketchyTimestamps(vicon.timestamps)
                vicon.pre_corrected_timestamps = vicon.timestamps
                vicon.timestamps = [ aslam.cv.Time(tt) for tt in corrected ]
                
                
            self.vicon = vicon
            self.db['vicon'] = vicon
        
    def processImageData(self, cameras, grid, useBagTime, correctTime=True):
        self.grid = grid
        #grid = acc.cv.GridCalibrationTarget(6,7,0.03,0.03,acc.cv.GridDetectorType.Checkerboard)
        self.cameras = []
        for cc in range(0,len(cameras)):
            cameraModel = cameras[cc]
            cameraTopic = cameraModel.topic
            obsTable = "{0}_observations".format(cameraTopic)
            imageTable = "{0}_images".format(cameraTopic)
            loaded = False
            if self.db.has_key(cameraTopic):
                print "Attempting to load previously processed image data for topic {0}".format(cameraTopic)
                camera = self.db[cameraTopic]
                camera.observations = self.db.getTable(obsTable)
                camera.images = self.db.getTable(imageTable)
                
                if (not len(camera.observations.keys()) == len(camera.timestamps))\
                        or (not len(camera.images.keys()) == len(camera.timestamps)):
                    print "Warning: mismatch between the timestamps ({0}) and the observations ({1}). Reloading observations".format(len(camera.timestamps),len(camera.observations.keys()))
                    print "     or: mismatch between the timestamps ({0}) and the images ({1}). Reloading observations".format(len(camera.timestamps),len(camera.images.keys()))
                    camera.observations.clearTable()
                    camera.images.clearTable()
                else:
                    self.cameras.append(camera)
                    loaded = True
                    print "Successfully loaded previously processed observations"
            if not loaded:
                print "Opening bag file: {0}".format(self.bagFileName)
                bag = rosbag.Bag(self.bagFileName, 'r')

                print "Processing camera {0}/{1} on topic {2}".format(cc+1,len(cameras), cameraTopic)
                print "Processing PnP data"
                # Next, get the checkerboard data
                timestamps, success, observations, bagTimestamps, correctedBagTimestamps, snappyTimestamps, images = loadCheckerboardData(bag, cameraTopic, cameraModel, grid, self.db.getTable(obsTable),self.db.getTable(imageTable))
                camera = Camera()
                camera.model = cameraModel
                camera.geometry = cameraModel.geometry
                if useBagTime:
                    camera.timestamps = correctedBagTimestamps
                else:
                    camera.timestamps = timestamps
                # Postprocess time
                if correctTime:
                    if len(snappyTimestamps) > 0:
                        print "Correcting the timestamps using MV hardware time"
                        tcorr = sm.DoubleTimestampCorrector()
                        camera.pre_corrected_timestamps = camera.timestamps
                        corrected = [ aslam.cv.Time(tcorr.correctTimestamp( float(snappyTimestamps[i]), camera.timestamps[i].to_sec() )) for i in range( 0, len(snappyTimestamps) ) ]
                        camera.timestamps = corrected
                        
                    else:
                        T,dT,med_dt,corrected,seqs,tcorr = correctSketchyTimestamps(camera.timestamps)
                        camera.pre_corrected_timestamps = camera.timestamps
                        camera.timestamps = [ aslam.cv.Time(tt) for tt in corrected ]
                        
                self.db.sync()
                print "Applying the corrected timestamps. This may take a while..."
                keys = observations.keys()
                for i in range(0,len(camera.timestamps)):
                    #print "correcting obs {0}".format(i)
                    if not i in keys:
                        print "{0} is not in the observation keys".format(i)
                        print keys
                    stamp = camera.timestamps[i]
                    obs = observations[i]
                    obs.setTime( acc.cv.Time(stamp.secs, stamp.nsecs) )
                    observations[i] = obs
                    i = i + 1
            
                camera.success = success
                camera.topic = cameraTopic
                print "Saving data for camera topic: {0}".format(cameraTopic)
                self.db[cameraTopic] = camera
                # Observations is a different table in the DB. Don't serialize it.
                camera.observations = observations
                self.cameras.append(camera)

def findInitialGuessForRigTransformation(cself, camera, skip=25):
    # 1. interpolate the vicon measurements for each image measurement.
    interpolateViconAtImageTimes(cself, camera, cself.vicon)
    # 2. Generate the input needed for the hand-eye calibration
    hccal = hc.StaticRigTransformationEstimator()
    #def setData(self, RListC, TListC, RListV, TListV):
    #    """take external data instead of using the pnp solver
    #    Args:
    #        RListC (list(np.array)): rotation matrices from chessboard to camera
    #        TListC (list(np.array)): translation vectors from chessboard to camera
    #        RListV (list(np.array)): rotation matrices vector from reference to rig
    #        TListV (list(np.array)): translation vectors from reference to rig
    #    """
    obs = camera.observations
    success = camera.success
    RListC = []
    TListC = []
    RListV = []
    TListV = []
    i = 0;
    # This loop just makes sure we only select from
    # successful poses
    # I took computer science, why isn't this loop simpler?
    while i < len(obs.keys()):
        while i < len(obs.keys()) and not success[i]:
            i = i + 1
        if i < len(obs.keys()) and success[i]:
            ob = obs[i]
            # NOTE: Should this be inverse?
            RListC.append( ob.T_t_c().C() )
            TListC.append( ob.T_t_c().t() )
            TT = camera.T_vicon_rig
            RListV.append( TT[i].C() )
            TListV.append( TT[i].t() )
        i = i + skip
    print "Running hand/eye calibration with {0} data points".format(len(RListC))
    hccal.setData( RListC, TListC, RListV, TListV )
    C_rig_camera, t_rig_camera = hccal.findHandEyeTransformation()
    C_vicon_checkerboard, t_vicon_checkerboard = hccal.findCheckerboardToWorld( C_rig_camera, t_rig_camera )
    cself.hccal = hccal
    return C_rig_camera, t_rig_camera, C_vicon_checkerboard, t_vicon_checkerboard

def evalInitialGuess(cself):
    T_rig_camera = []
    T_vicon_checkerboard = []
    R = range(10,110)
    for i in R:
        print "{0}/110".format(i)
        C_rig_camera, t_rig_camera, C_vicon_checkerboard, t_vicon_checkerboard = findInitialGuessForRigTransformation(cself,skip=i)
        T_rig_camera.append( sm.Transformation( buildT(C_rig_camera, t_rig_camera ) ) )
    tt = buildTransformationCurves(T_rig_camera)
    pl.plot(R,tt)
    pl.xlabel('sample')
    return T_rig_camera
    

def interpolateViconAtImageTimes(cself, camera, vicon):
    viconIdx = 0
    camera.T_vicon_rig = []
    # assume the list of vicon measurements is sorted
    for camIdx in range(0,len( camera.observations.keys() ) ):
        T_vicon_rig = sm.Transformation()
        # find a vicon point that is older than the camera point
        while viconIdx < len(vicon.timestamps) and \
                vicon.timestamps[viconIdx] < camera.timestamps[camIdx]:
            viconIdx = viconIdx + 1
        if viconIdx == 0 or viconIdx >= len(vicon.timestamps):
            sm.logWarn("Camera point {0} was outside of the range of the vicon. Setting the observation to false".format(camIdx))
            camera.success[camIdx] = False
        else:
            # Check the temporal distance between vicon points:
            vt0 = vicon.timestamps[viconIdx-1].to_sec()
            T0 = vicon.T_vicon_rig[viconIdx-1]
            vt1 = vicon.timestamps[viconIdx].to_sec()
            T1 = vicon.T_vicon_rig[viconIdx]
            dvt = (vt1 - vt0)
            if dvt > 0.1:
                sm.logWarn("Camera point {0} seems to be in a vicon dropout of {1} seconds. Setting the observation to false".format(camIdx,dvt))
                camera.success[camIdx] = False
            else:
                # actually interpolate
                #T_vicon_rig = sm.interpolateTransformations(T0, vt0, T1, vt1, camera.timestamps[camIdx].to_sec())
                T_vicon_rig = sm.interpolateTransformations(T0, vt0, T1, vt1, camera.timestamps[camIdx].to_sec())
                #print "--"
                #print T_vicon_rig
                #print T0
                T_vicon_rig = T0
        camera.T_vicon_rig.append(T_vicon_rig)

def buildTransformationCurvesQuat(TT):
    t = []
    previousQuat = TT[0].q()
    for i in range(0,len(TT)):
        T = TT[i]
        q = T.q()
        pos = np.linalg.norm( q - previousQuat)
        neg = np.linalg.norm( (-q) - previousQuat)
        if neg < pos:
            q = -q
        previousQuat = q;
        t.append( np.hstack([ q, T.t() ] ) )

def buildTransformationCurves(TT, rot=None):
    t = []
    if rot is None:
        rot = sm.RotationVector()
    previousRotationVector = rot.rotationMatrixToParameters(TT[0].C())
    for i in range(0,len(TT)):
        T = TT[i]
        r = rot.rotationMatrixToParameters(T.C())
        if previousRotationVector is not None:
            angle = np.linalg.norm(r)
            axis = r/angle
            best_r = r
            best_dist = np.linalg.norm( best_r - previousRotationVector)
            for s in range(-4,5):
                aa = axis * (angle + np.math.pi * 2.0 * s)
                dist = np.linalg.norm( aa - previousRotationVector )
                if dist < best_dist:
                    best_r = aa
                    best_dist = dist
                    r = best_r;
        previousRotationVector = r;
        t.append( np.hstack([ r, T.t() ] ) )

    return np.array(t)


    return np.array(t)
def getCameraTransformations(cself):
    timestamps = []
    T_target_camera = []
    T_vicon_rig = []
    for i in range(0,len(cself.camera.observations.keys())):
        if cself.camera.success[i]:
            timestamps.append(cself.camera.timestamps[i])
            T_target_camera.append(cself.camera.observations[i].T_t_c())
            T_vicon_rig.append(cself.camera.T_vicon_rig[i])
    return timestamps, T_target_camera, T_vicon_rig

    
def addDesignVariables(cself, problem, dvc, setActive=True):
    for i in range(0,dvc.numDesignVariables()):
        dv = dvc.designVariable(i)
        dv.setActive(setActive)
        problem.addDesignVariable(dv)


def createDesignVariables(cself, initial, cameraTopics=None, estimateT_rig_camera=False, estimateT_vicon_target=False, estimateGrid=False, estimateShutter=False, estimateProjection=False, estimateDistortion=False, estimateTimeOffset=False):
    if cameraTopics is None:
        cameraTopics = initial.cameraTopic
    cself.problem = acc.opt.OptimizationProblem()
    cself.designVariables = DesignVariables()
    # Create T_vicon_rig, the B-Splne pose design variable
    # This should always be enabled.
    cself.designVariables.T_vicon_rig = asp.BSplinePoseDesignVariable(initial.T_vicon_rig)
    addDesignVariables(cself, cself.problem, cself.designVariables.T_vicon_rig)
    
    cself.designVariables.cameras = []
    cself.designVariables.T_rig_camera = []
    cself.designVariables.viconTimeToCameraTime = []
    cself.designVariables.cameraTopic = []
    cself.designVariables.cameraGeometry = []
    
    for T_rig_camera, geometry, topic, vct in zip(initial.T_rig_camera, initial.cameraGeometry, initial.cameraTopic, initial.viconTimeToCameraTime):
        camActive = topic in cameraTopics
        # Create T_rig_camera, a transformation
        dv = acc.opt.TransformationDv(T_rig_camera)
        cself.designVariables.T_rig_camera.append( dv )
        addDesignVariables(cself, cself.problem, dv, setActive=(estimateT_rig_camera))
        cself.designVariables.cameraTopic.append(topic)
        cself.designVariables.cameraGeometry.append(geometry)
        # This will automatically set itself active/not active based on the flags.
        camDv = aslam.backend.cv.CameraGeometryDesignVariableContainer( geometry, estimateProjection, estimateDistortion, estimateShutter )
        cself.designVariables.cameras.append(camDv)
        # Create a time delay variable.
        vctdv = acc.opt.Scalar(vct)
        vctdv.setActive( estimateTimeOffset and camActive )
        vctdv.setScaling( 1.0 )
        cself.designVariables.viconTimeToCameraTime.append(vctdv)
        dv = camDv.getDesignVariable()
        dv.setActive(camActive)
        cself.problem.addDesignVariable(dv)
        cself.problem.addDesignVariable( vctdv )
        #addDesignVariables(cself, cself.problem, dv, setActive=(estimateT_rig_camera and camActive))
    # Create variables for the target points.
    cself.designVariables.target = acc.cvb.GridCalibrationTargetDesignVariableContainer(initial.target, estimateGrid)
    for dv in cself.designVariables.target.getDesignVariables():
        cself.problem.addDesignVariable(dv)
    # Create T_vicon_target, a transformation
    cself.designVariables.T_vicon_target = acc.opt.TransformationDv(initial.T_vicon_target)
    addDesignVariables(cself, cself.problem, cself.designVariables.T_vicon_target, setActive=(estimateT_vicon_target and not estimateGrid))

    # Create expressions for all points expressed in the target frame
    T_vicon_target = cself.designVariables.T_vicon_target.toExpression()
    cself.designVariables.p_vicon = [ T_vicon_target * cself.designVariables.target.getPoint(i).toHomogeneousExpression() for i in range(0,cself.grid.size()) ]

def addReprojectionErrors(cself, camera, cameraDv, T_rig_camera, viconTimeToCameraTime, doMEstimator=False, mestimator=None, sigma=0.25, timeOffsetPadding=1.0):
    # cache an expression of the time offset
    v2c = viconTimeToCameraTime.toExpression()
    # cache an expression for the rig to camera transformation
    T_camera_rig = T_rig_camera.toExpression().inverse()
    
    # cache the reprojection error type
    camera.reprojectionErrors = []
    camera.reprojectionErrorTimes = []
    camera.reprojectionErrorIndices = []

    invR = np.eye(2) * (1.0 / (sigma * sigma))
    if mestimator is None:
        mestimator = acc.opt.HuberMEstimator(3.5)
    camera.mestimator = mestimator
    camera.doMEstimator = doMEstimator
    nullcount = 0
    # HACK! Remove the first and last observations to 
    # stay within the bounds
    remove = 0
    tmin=None
    tmax=None
    for o in range(remove,len(camera.observations.keys())-remove):
        obs = camera.observations[o]
        target = obs.target()
        if target is None:
            nullcount = nullcount + 1
            #print "Found a null target in observation {0}".format(o)
        else:
            for i in range(0,target.size()):
                success, y = obs.imagePoint(i)
                # If the point was a good observation
                if success:
                    # Get the time of this observation as an expression
                    # This returns an expression from the camera design variable
                    # it will depend on the shutter if we are doing that.
                    temporalOffset = cameraDv.temporalOffset( y )
                    ts = v2c + obs.time().toSec() + temporalOffset
                    # print "--"
                    # print (ts.toScalar() + timeOffsetPadding)
                    # print cself.designVariables.T_vicon_rig.spline().t_max()
                    # print (ts.toScalar() - timeOffsetPadding)
                    # print cself.designVariables.T_vicon_rig.spline().t_min()
                    inbounds = (ts.toScalar() + timeOffsetPadding) < cself.designVariables.T_vicon_rig.spline().t_max() and\
                        (ts.toScalar() - timeOffsetPadding) > cself.designVariables.T_vicon_rig.spline().t_min()
                    # print inbounds
                    if inbounds:
                        if tmin is None or tmax is None:
                            tmin = ts.toScalar()
                            tmax = ts.toScalar()
                        else:
                            tmin = min(tmin,ts.toScalar())
                            tmax = max(tmax,ts.toScalar())
                        # Get the pose expression at this time
                        T_vicon_rig = cself.designVariables.T_vicon_rig.transformationAtTime(ts,timeOffsetPadding,timeOffsetPadding)
                        T_camera_vicon = T_camera_rig * T_vicon_rig.inverse()
                        p_camera = T_camera_vicon * cself.designVariables.p_vicon[i]
                        re = cameraDv.createReprojectionError(y, invR, p_camera)
                        re.evaluateError()                    
                        if camera.doMEstimator:
                            re.setMEstimatorPolicy( mestimator )
                        camera.reprojectionErrors.append(re)
                        camera.reprojectionErrorTimes.append(ts)
                        camera.reprojectionErrorIndices.append(o)
                        
    camera.tmin = tmin
    camera.tmax = tmax
    print "Found {0}/{1} grid observations producing {2} reprojection errors".format(len(camera.observations.keys()) - nullcount, len(camera.observations.keys()), len(camera.reprojectionErrors))
    for re in camera.reprojectionErrors:
        cself.problem.addErrorTerm(re)
        pass

def addViconErrorTerms(cself, sigmarot=sm.deg2rad(0.1), sigmatrans=0.001,addToProblem=True):
    wrot = 1.0/(sigmarot * sigmarot)
    wtrans = 1.0/(sigmatrans * sigmatrans)
    errors = []
    sp_T_vicon_rig = cself.designVariables.T_vicon_rig
    spline = sp_T_vicon_rig.spline()
    for i in range(0,len(cself.vicon.T_vicon_rig)):
        # Create a transformation expression for this time
        ts = cself.vicon.timestamps[i].to_sec()
        T_vicon_rig = cself.vicon.T_vicon_rig[i]
        if ts > spline.t_min() and ts < spline.t_max():
            hat_T_vicon_rig = sp_T_vicon_rig.transformation(ts)
            err = acc.opt.ErrorTermTransformation(hat_T_vicon_rig, T_vicon_rig, wrot, wtrans);
            err.evaluateError()
            errors.append(err)
    cself.vicon.errors = errors
    if(addToProblem):
        for re in errors:
            cself.problem.addErrorTerm(re)
    print "Found {0} vicon error terms".format(len(errors))

def findCameraMinMaxTimes(cself,splineOrder=4, knotsPerSecond=15.0):
    minTime = min(cself.camera.timestamps).to_sec()
    maxTime = max(cself.camera.timestamps).to_sec() + cself.camera.geometry.temporalOffset(np.array( [cself.camera.geometry.projection().ru(), cself.camera.geometry.projection().rv()]) ).toSec()
    return (minTime,maxTime)

def initPoseSplineFromObservations(cself, camera, T_rig_camera, T_vicon_target, viconTimeToCameraTime, splineOrder, knotsPerSecond, splineRotationParameterization=None):
    # initialize the pose spline
    print "Initializing the pose spline"
    timestamps = []
    T_vicon_rig = []
    T_camera_rig = np.linalg.inv(T_rig_camera.T())
    for i in range(0, len(camera.observations.keys())):
        obs = camera.observations[i]
        if camera.success[i]:
            timestamps.append( obs.time() + viconTimeToCameraTime )
            T_target_camerak = obs.T_t_c().T()
            T_vicon_camerak = np.dot(T_vicon_target.T(), T_target_camerak)
            T_vicon_rigk = np.dot(T_vicon_camerak, T_camera_rig)
            T_vicon_rig.append( sm.Transformation(T_vicon_rigk)  )
    # Add one guy on the end to make sure we can get the image measurements over there
    #tt = timestamps[-1] + 
    #timestamps.append( tt )
    #T_vicon_rig.append( sm.Transformation(T_vicon_rigk) )

    spline_T_vicon_rig, stimes, sparams = initPoseSpline(timestamps, T_vicon_rig, splineOrder=splineOrder, knotsPerSecond=knotsPerSecond, splineRotationParameterization=splineRotationParameterization)
    return spline_T_vicon_rig, T_vicon_rig, stimes, sparams
def runLocalization(cself, initial, camera, splineOrder=4, knotsPerSecond=15.0, rotationMotion=1.0, translationMotion=1e-2, motionOrder=2, lineDelay=None, splineRotationParameterization=None):
    found = -1
    for i in range(0,len(initial.cameraTopic)):
        if initial.cameraTopic[i] == camera.topic:
            found = i
            break
    if found < 0:
        raise RuntimeError("Unable to find camera topic {0} in the initial guesses {1}".format(camera.topic, initial.cameraTopic))

    # This is clumsy. Better in the end to use a struct.
    T_rig_camera = initial.T_rig_camera[i]
    topic = initial.cameraTopic[i]
    viconTimeToCameraTime = aslam.cv.Duration(initial.viconTimeToCameraTime[i])
    geometry = initial.cameraGeometry[i]

    if not lineDelay is None:
        print "Got a request to set the line delay to {0}".format(lineDelay)
        if geometry.minimalDimensionsShutter() != 1:
            print "Error: This camera has an unexpected number of shutter parameters. Expected 1, got {0}".format(geometry.minimalDimensionsShutter())
        else:
            # Set the line delay
            shutter = geometry.shutter()
            shutter.setParameters( np.array([lineDelay]))

    
    T_vicon_rig, orig_T_vicon_rig, stimes, sparams = initPoseSplineFromObservations(cself, camera, T_rig_camera, initial.T_vicon_target, viconTimeToCameraTime, splineOrder, knotsPerSecond, splineRotationParameterization=splineRotationParameterization)

    initial.T_vicon_rig = T_vicon_rig
    # Create the design variables.
    print "Creating Design Variables"
    createDesignVariables(cself, initial, cameraTopics=[topic], estimateGrid=False, estimateShutter=False, estimateTimeOffset=False)
    # Create the error terms
    for camera, cameraDv, T_rig_camera, vct in zip(cself.cameras, cself.designVariables.cameras, cself.designVariables.T_rig_camera, cself.designVariables.viconTimeToCameraTime):
        if camera.topic == topic:
            print "Creating Reprojection Errors from topic {0}".format(camera.topic)
            addReprojectionErrors(cself,camera,cameraDv,T_rig_camera, vct, timeOffsetPadding = 0.0)
    
    addViconErrorTerms(cself, addToProblem=False)
    addMotionTerms(cself, rotationMotion, translationMotion, motionOrder)

    print "Creating Optimizer"
    options = acc.opt.OptimizerOptions()
    options.maxIterations = 10
    options.verbose = True
    options.linearSolver = 'spqr'
    # Something is wrong. This seems to fix it. Blerg.
    options.resetSolverEveryIteration=True
    options.levenbergMarquardtLambdaInit = 100.0
    cself.optimizer = acc.opt.Optimizer(options)
    cself.optimizer.setProblem(cself.problem)
    print "Initial Guesses:"
    printResults(cself)
    tbefore = time.time();
    cself.optimizer.optimizeDogLeg()
    timing = time.time() - tbefore;
    cself.optimizationTime = timing;
    print "After optimization:"
    printResults(cself)

def addMotionTerms(cself, rotation=1.0, translation=1e-2, order=2):
    # add Smoothing terms
    W = np.eye(6)
    W[:3,:3] *= translation #1e-3
    W[3:,3:] *= rotation #1
    
    if order > 0:
        print "Adding motion term of order {0}:\n{1}".format(order,W)
        cself.motionErrorTerm = asp.BSplineMotionError(cself.designVariables.T_vicon_rig, W, order)
        cself.problem.addErrorTerm(cself.motionErrorTerm)
    else:
        print "Skipping the motion term"

def setSameLimits(fnos):
    master = fnos[0]
    f=pl.figure(master)
    xlim = pl.xlim()
    ylim = pl.ylim()
    for fno in fnos:
        f = pl.figure(fno)
        pl.xlim(xlim)
        pl.ylim(ylim)

def setSameXLimits(fnos):
    master = fnos[0]
    f=pl.figure(master)
    xlim = pl.xlim()
    for fno in fnos:
        f = pl.figure(fno)
        pl.xlim(xlim)

def setSameYLimits(fnos):
    master = fnos[0]
    f=pl.figure(master)
    ylim = pl.ylim()
    for fno in fnos:
        f = pl.figure(fno)
        pl.ylim(ylim)


def runCalibration(cself, estimateGrid=False, estimateShutter=True, estimateIntrinsics=False, knotsPerSecond=15.0, cameraTopics=None):
    initialTimeOffset = 0.0
    cself.initial = Estimate()
    cself.initial.target = cself.grid
    cself.initial.T_rig_camera = []
    cself.initial.viconTimeToCameraTime = []
    cself.initial.cameraTopic = []
    cself.initial.cameraGeometry = []
    for camera in cself.cameras:
        C_rig_camera, t_rig_camera, C_vicon_target, t_vicon_target = findInitialGuessForRigTransformation(cself, camera, skip=25)
        cself.initial.T_rig_camera.append(sm.Transformation( buildT( C_rig_camera, t_rig_camera ) ))
        cself.initial.T_vicon_target = sm.Transformation( buildT( C_vicon_target, t_vicon_target ) )
        cself.initial.viconTimeToCameraTime.append(0.0)
        cself.initial.cameraTopic.append(camera.topic)
        cself.initial.cameraGeometry.append(camera.geometry)
    # initialize the pose spline
    print "Initializing the pose spline"
    cself.initial.T_vicon_rig, stimes, sparams = initPoseSpline(cself.vicon.timestamps, cself.vicon.T_vicon_rig, knotsPerSecond=knotsPerSecond)
    if cameraTopics is None:
        cameraTopics = cself.initial.cameraTopic

    # Create the design variables.
    print "Creating Design Variables"
    createDesignVariables(cself, cself.initial, cameraTopics=cameraTopics, estimateT_rig_camera=True, estimateT_vicon_target=True, estimateGrid = estimateGrid, estimateShutter = estimateShutter, estimateTimeOffset=True, estimateProjection=estimateIntrinsics, estimateDistortion=estimateIntrinsics)
    # Create the error terms
    for camera, cameraDv, T_rig_camera, vct in zip(cself.cameras, cself.designVariables.cameras, cself.designVariables.T_rig_camera, cself.designVariables.viconTimeToCameraTime):
        if camera.topic in cameraTopics:
            print "Creating Reprojection Errors from topic {0}".format(camera.topic)
            addReprojectionErrors(cself,camera,cameraDv,T_rig_camera, vct)
    print "Creating Vicon Error Terms"
    addViconErrorTerms(cself)
    print "Creating Optimizer"
    if True:
        options = acc.opt.Optimizer2Options()
        options.verbose = True
        options.linearSolver = acc.opt.SparseCholeskyLinearSystemSolver()
        options.trustRegionPolicy = acc.opt.DogLegTrustRegionPolicy()
        cself.optimizer = acc.opt.Optimizer2(options)
        cself.optimizer.setProblem(cself.problem)

        print "Initial Guesses:"
        printResults(cself)
        cself.optimizer.optimize()
        print "After optimization:"
        printResults(cself)
    else:
        print "Creating Optimizer"
        options = acc.opt.OptimizerOptions()
        options.maxIterations = 40
        options.verbose = True
        options.linearSolver = 'spqr'
        # Something is wrong. This seems to fix it. Blerg.
        options.resetSolverEveryIteration=True
        options.levenbergMarquardtLambdaInit = 100.0
        cself.optimizer = acc.opt.Optimizer(options)
        cself.optimizer.setProblem(cself.problem)
        print "Initial Guesses:"
        printResults(cself)
        cself.optimizer.optimizeDogLeg()
        print "After optimization:"
        printResults(cself)

def initPoseSpline(timestamps, transformations, knotsPerSecond=15.0, smoothing=1e-6, splineOrder=4, splineRotationParameterization=None):
    '''Initialize a pose spline based on an array of timestamps and transformation matrices'''
    print "Initializing the pose spline with {0} knots per second and order {1}".format(knotsPerSecond, splineOrder)
    if splineRotationParameterization is None:
       splineRotationParameterization = sm.RotationVector() 
       
    print "Initializing the spline with rotation={0}".format(type(splineRotationParameterization))
    bsplinePose = bsplines.BSplinePose( splineOrder, splineRotationParameterization)
            
    # initialise spline:  (6 rows, n cols)
    gsPositions = len(transformations)
    splinePoses = np.zeros((6, gsPositions+2))

    # Pad the spline time at the start and end by "timeMargin"
    R = transformations[0].C()
    T = transformations[0].t()
    # convert rotation matrix to parametrisation
    r = splineRotationParameterization.rotationMatrixToParameters(R)
    splinePoses[:,0] = np.concatenate((T,r))    # add the same position as the last frame
    previousRotationVector = r
    # create the corresponding time vector:
    t0 = timestamps[0]  # start at time 0 and in seconds:
    splineTimes = [t0.to_sec()]
    
    ## Loop Positions
    for i in range(0,len(transformations)):
        R = transformations[i].C()
        T = transformations[i].t()
        # convert rotation matrix to parametrisation
        r = splineRotationParameterization.rotationMatrixToParameters(R)
        # remove ambiguity in RotationVector parametrisation
        if previousRotationVector is not None:
            angle = np.linalg.norm(r)
            axis = r/angle
            best_r = r
            best_dist = np.linalg.norm( best_r - previousRotationVector)
            for s in range(-3,4):
                aa = axis * (angle + np.math.pi * 2.0 * s)
                dist = np.linalg.norm( aa - previousRotationVector )
                if dist < best_dist:
                    best_r = aa
                    best_dist = dist
                    r = best_r;
        
        
        # set the new previous one
        previousRotationVector = r
        # add to matrix:
        splinePoses[:,i+1] = np.concatenate((T,r))
        splineTimes.append(timestamps[i].to_sec())
    # add the padding at the end
    splineTimes.append(timestamps[-1].to_sec())
    splinePoses[:,-1] = splinePoses[:,-2]
    seconds = splineTimes[-1] - splineTimes[0]
    numKnots = int(knotsPerSecond * seconds)
    bsplinePose.initPoseSplineSparse( np.array(splineTimes), splinePoses, numKnots, smoothing)
    return (bsplinePose, splineTimes, splinePoses)
def plotReprojectionError(cself,fno=1):
    rerrs=[]
    for re in cself.camera.reprojectionErrors:
        re.evaluateError()
        rerrs.append(re.error())
    rerrs = np.array(rerrs)
    
    f = pl.figure(fno)
    f.clf()
    pl.scatter(rerrs[:,0],rerrs[:,1])
    pl.xlabel('error u (pixels)')
    pl.ylabel('error v (pixels)')
    pl.grid('on')
    pl.axis('equal')


def plotCheckerboardSequence(camera, T_world_body, T_body_camera, T_world_target, target, observations, cameraGeometry, imgTimeToSplineTime=0.0,rescale=1.0):
    #pl.close('all')
    # Build the points in the world frame
    f = pl.figure(1)
    remove = 2
    # HACK! Remove observations
    for o in range(remove,len(observations.keys())-remove):
        f.clf()
        obs = observations[o]
        target = obs.target()
        imgStamp = obs.time().toSec()
        # Generate the predicted checkerboard locations.
        Y = []
        hatY = []
        for i in range(0,target.size()):
            success, y = obs.imagePoint(i)
            if success:
                obsStamp = imgStamp + cameraGeometry.temporalOffset( y ).toSec() + imgTimeToSplineTime
                p_world = np.dot(T_world_target.T(), sm.toHomogeneous(target.point(i)));
                T_world_camera = np.dot(T_world_body.transformation( obsStamp ),T_body_camera.T())
                T_camera_world = np.linalg.inv(T_world_camera)
                p_camera = np.dot(T_camera_world, p_world)
                hat_y = cameraGeometry.homogeneousToKeypoint(p_camera)
                Y.append(y)
                hatY.append(hat_y)
        I = obs.getImage()
        pl.imshow(I,cmap=pl.cm.gray)
        XL = pl.xlim()
        YL = pl.ylim()
        for i in range(0,len(Y)):
            y = Y[i]
            haty = hatY[i]
            dy = rescale * (haty-y);
            
            pl.plot(y[0],y[1],'rx')
            pl.plot(haty[0],haty[1],'b.')
            pl.plot([y[0],y[0]+dy[0]],[y[1],y[1] + dy[1]],'c-', lw=2, alpha=0.7)
        pl.xlim(XL)
        pl.ylim(YL)
        pl.draw()
        pl.show()
                

def plotCalibratorSequence(cself, rescale=1.0):
    plotCheckerboardSequence(cself.designVariables.T_vicon_rig.spline(),
                             sm.Transformation(cself.designVariables.T_rig_camera.T()),
                             sm.Transformation(cself.designVariables.T_vicon_target.T()),
                             cself.grid,
                             cself.camera.observations,
                             cself.camera.geometry,
                             cself.designVariables.viconTimeToCameraTime.toValue(),
                             rescale=rescale);
def evalError(x):
    x.evaluateError()
    return x.error()
    
def saveCalibrationResults(cself):
    calibrated = Estimate()
    calibrated.T_vicon_target = sm.Transformation(cself.designVariables.T_vicon_target.T())
    calibrated.target = cself.grid
    calibrated.T_rig_camera = []
    calibrated.cameraGeometry = []
    calibrated.cameraTopic = []
    calibrated.viconTimeToCameraTime = []
    for T_rig_camera, camera, vct in zip(cself.designVariables.T_rig_camera, cself.cameras, cself.designVariables.viconTimeToCameraTime):
        calibrated.T_rig_camera.append( sm.Transformation(T_rig_camera.T()) )
        calibrated.cameraGeometry.append( camera.geometry )
        calibrated.viconTimeToCameraTime.append(vct.toScalar())
        calibrated.cameraTopic.append( camera.topic )
    cself.calibrated = calibrated
    cself.db['calibrated'] = calibrated
    cself.db.sync()

def propagateNewCalibration(cself):
    for i in range(0,len(cself.cameras)):
        cself.cameras[i].geometry = cself.calibrated.cameraGeometry[i]
        

def loadCalibrationResults(cself, estimateGrid=False, estimateShutter=False, cameraTopics=None, forceRecalibrate = False, estimateIntrinsics=False, knotsPerSecond=15.0):
    print "Checking for previously processed calibration"
    if cself.db.has_key('calibrated') and not forceRecalibrate:
        print "Loading previous calibration"
        calibrated = cself.db['calibrated']
        for camera in cself.cameras:
            found = False
            for geometry, topic in zip(calibrated.cameraGeometry,calibrated.cameraTopic):
                if topic == camera.topic:
                    found = True
                    camera.geometry = geometry
            if not found:
                print "Warning: Unable to find calibration data for the topic {0}".format(camera.topic)
        cself.grid = calibrated.target
        cself.T_vicon_target = calibrated.T_vicon_target
        cself.calibrated = calibrated
    else:
        print "No previous calibration found. Running calibration"
        runCalibration(cself, estimateGrid=estimateGrid, estimateShutter=estimateShutter, cameraTopics=cameraTopics, estimateIntrinsics=estimateIntrinsics, knotsPerSecond=knotsPerSecond)
        saveCalibrationResults(cself)
    propagateNewCalibration(cself)

def loadCalibrationResultsFromFile(cself, filename):
    print "Loading calibration results from {0}".format(filename)
    if not os.path.isfile(filename):
        raise RuntimeError("The calibration file {0} does not exist.".format(filename))
    db = sww.ShelveDb(filename)
    if db.has_key('calibrated'):
        cself.calibrated = db['calibrated']
    else:
        raise RuntimeError("There is no calibration data stored in the file {0}.".format(filename))
    propagateNewCalibration(cself)

def saveCalibrationResultsToFile(cself, filename):
    db = sww.ShelveDb(filename)
    db['calibrated'] = cself.calibrated
    db.sync()

def saveOutput(output, filename, tag='output'):
    print "saving output to file {0}".format(filename)
    db = sww.ShelveDb(filename)
    db[tag] = output
    db.sync()

def loadOutput(filename, tag='output'):
    print "loading output to file {0}".format(filename)
    db = sww.ShelveDb(filename)
    if not db.has_key(tag):
        raise RuntimeError("There is no output data stored under tag '{0}' in the file '{0}'.".format(tag,filename))
    return db[tag]


def printResults(cself):
    print "T_vicon_target:"
    print cself.designVariables.T_vicon_target.T()

    ev = np.array([evalError(ve) for ve in cself.vicon.errors])
    mev = np.mean(ev,0)
    sev = np.std(ev,0)
    print "vicon translation errors: {0} +- {1}".format(mev[0:3],sev[0:3])
    print "vicon rotation errors:    {0} +- {1}".format(mev[3:6],sev[3:6])
    for T_rig_camera, camera, vct in zip(cself.designVariables.T_rig_camera, cself.cameras, cself.designVariables.viconTimeToCameraTime):
        if hasattr(camera,"reprojectionErrors"):
            print "Camera {0}".format(camera.topic)
            print "  viconTimeToCameraTime: {0}".format(vct.toScalar())
            print "  T_rig_camera:"
            print T_rig_camera.T()
            er = np.array([evalError(re) for re in camera.reprojectionErrors])
            mer = np.mean(er,0)
            ser = np.std(er,0)
            print "  reprojection error: {0} +- {1}".format(mer,ser)
            if camera.geometry.minimalDimensionsShutter() > 0:
                ld = camera.geometry.getParameters(False,False,True)[0,0]
                print "  line delay: {0}".format( ld ) 
            print "  intrinsics: {0}".format(camera.geometry.getParameters(True,True,False).T)

def findMissingMeasurementIntervals(cself, offset=None):
    startTime = None
    if offset is None:
        offset = cself.designVariables.viconTimeToCameraTime.toScalar()
    # This will be a list of intervals with missing measurements
    lastTime=cself.camera.observations[0].time()
    missing = []
    for oidx in range(0,len(cself.camera.observations.keys())):
        obs = cself.camera.observations[oidx]
        success = False
        for i in range(0,obs.target().size()):
            success_i, p = obs.imagePoint(i)
            # success if any of the points were found.
            success = success or success_i
        if success:
            if not startTime is None:
                # Here we have found the end of an interval!
                missing.append( (startTime.toSec() + offset, obs.time().toSec() + offset ) )
                # Look for a new start time
                startTime = None
        else:
            if startTime is None:
                # Here we have found the start of a new sequence
                startTime = lastTime#obs.time()
        lastTime = obs.time()
    # Catch the very last interval
    if not startTime is None:
        # Here we have found the end of an interval!
        missing.append( (startTime.toSec() + offset, obs.time().toSec() + offset ) )
        
    return missing

def findMeasurementIntervals(cself, offset=None):
    startTime = None
    if offset is None:
        offset = cself.designVariables.viconTimeToCameraTime.toScalar()
    # This will be a list of intervals with missing measurements
    measurements = []
    for oidx in range(0,len(cself.camera.observations.keys())):
        obs = cself.camera.observations[oidx]
        success = False
        for i in range(0,obs.target().size()):
            success_i, p = obs.imagePoint(i)
            # success if any of the points were found.
            success = success or success_i
        if success:
            startTime = obs.time().toSec() + offset
            endTime = obs.time().toSec() + offset + cself.camera.geometry.temporalOffset( np.array([0, cself.camera.geometry.projection().rv()])).toSec()
            # Here we have found the end of an interval!
            measurements.append( (startTime, endTime) )
        
    return measurements

def plotIntervals(cself, axes, intervals):
    # Plot the transparent areas where there are no measurements
    for ax in axes:
        ylim = ax.get_ylim()
        for interval in intervals:
            rect = pl.Rectangle( (interval[0]-cself.camera.tmin,ylim[0]), interval[1]-interval[0], ylim[1] - ylim[0], color=(0,0,0,0.2), fill=True)
            ax.add_patch(rect)

def plotCameraError(cself,fno=1):
    axes = plotError(cself, cself.camera.tmin, cself.camera.tmax, fno=fno)
    
    # Plot the transparent areas where there are measurements
    intervals  = findMeasurementIntervals(cself)
    plotIntervals(cself, axes, intervals)

    # Plot the transparent areas where there are no measurements
    # for ax in axes:
    #     ylim = ax.get_ylim()
    #     missing = findMissingMeasurementIntervals(cself)
    #     for interval in missing:
    #         rect = pl.Rectangle( (interval[0]-cself.camera.tmin,ylim[0]), interval[1]-interval[0], ylim[1] - ylim[0], color=(0,0,0,0.2), fill=True)
    #         ax.add_patch(rect)

def plotError(cself, tmin=None, tmax=None, spline=None, fno=1):
    # For every vicon measurement, pull a measurement from the spline.
    if spline is None:
        spline_T_vicon_rig = cself.designVariables.T_vicon_rig.spline()
    else:
        spline_T_vicon_rig = spline
    if tmin is None:
        tmin = spline_T_vicon_rig.t_min();
    if tmax is None:
        tmax = spline_T_vicon_rig.t_max();

    timestamps = [ t for t in cself.vicon.timestamps ]
    est_T_vicon_rig = [ spline_T_vicon_rig.transformation( t.to_sec() ) for t in timestamps ]
    C_err = [ sm.R2AxisAngle(np.dot(est_T_vicon_rig[i][0:3,0:3], cself.vicon.T_vicon_rig[i].inverse().C())) for i in range(0,len(est_T_vicon_rig)) ]

    t_err = [ est_T_vicon_rig[i][0:3,3] - cself.vicon.T_vicon_rig[i].t() for i in range(0,len(est_T_vicon_rig)) ]
    C_err_trimmed = []
    t_err_trimmed = []
    timestamps_trimmed = []
    # Now filter by time
    for i in range(0,len(cself.vicon.timestamps)):
        tt = cself.vicon.timestamps[i].to_sec()
        if tt >= tmin and tt <= tmax:
            C_err_trimmed.append(C_err[i])
            t_err_trimmed.append(t_err[i])
            timestamps_trimmed.append(tt)

    times = np.array(timestamps_trimmed) - timestamps_trimmed[0]
    t2_err = np.array([ [np.linalg.norm(t_err_trimmed[i]), np.linalg.norm(C_err_trimmed[i]) ] for i in range(0,len(t_err_trimmed)) ])
    t3_err = np.vstack(t_err_trimmed)
    c3_err = np.vstack(C_err_trimmed)
    xlim = np.array([ times[0], times[-1]])
    axes = []
    # The 3x1 figure errors
    f1 = pl.figure(fno)
    f1.clf()
    pl.title('component errors')
    ax = pl.subplot(211)
    axes.append(ax)
    pl.plot( times, t3_err[:,0]*1e3,'r-',lw=2)
    pl.plot( times, t3_err[:,1]*1e3,'g-',lw=2)
    pl.plot( times, t3_err[:,2]*1e3,'b-',lw=2)
    pl.xlabel('time (s)')
    pl.ylabel('translation error [mm]')
    pl.xlim(xlim)
    pl.grid('on')
    ax = pl.subplot(212)
    axes.append(ax)
    pl.plot( times, c3_err[:,0],'r-',lw=2)
    pl.plot( times, c3_err[:,1],'g-',lw=2)
    pl.plot( times, c3_err[:,2],'b-',lw=2)
    pl.xlabel('time (s)')
    pl.ylabel('rotation error')
    pl.xlim(xlim)
    pl.grid('on')

    # The 3x1 figure errors
    f1 = pl.figure(fno+1)
    f1.clf()
    pl.title('norm errors')
    ax = pl.subplot(211)
    axes.append(ax)
    pl.plot( times, t2_err[:,0]*1e3, 'r-', lw=2)
    pl.xlabel('time (s)')
    pl.ylabel('translation error [mm]')
    pl.xlim(xlim)
    pl.grid('on')
    ax = pl.subplot(212)
    axes.append(ax)
    pl.plot( times, np.degrees(t2_err[:,1]), 'r-', lw=2)
    pl.xlabel('time (s)')
    pl.ylabel('rotation error [deg]')
    pl.xlim(xlim)
    pl.grid('on')
    return axes

def plotCurves(cself, tmin=None, tmax=None, spline=None, fno=None):
    # For every vicon measurement, pull a measurement from the spline.
    if spline is None:
        spline_T_vicon_rig = cself.designVariables.T_vicon_rig.spline()
    else:
        spline_T_vicon_rig = spline
    if tmin is None:
        tmin = spline_T_vicon_rig.t_min();
    if tmax is None:
        tmax = spline_T_vicon_rig.t_max();

    timestamps = [ t for t in cself.vicon.timestamps ]
    est_T_vicon_rig = [ spline_T_vicon_rig.transformation( t.to_sec() ) for t in timestamps ]
    estX = np.array([ est_T_vicon_rig[i][0:3,3] for i in range(0,len(est_T_vicon_rig)) ]).T
    vicX = np.array([ cself.vicon.T_vicon_rig[i].T()[0:3,3] for i in range(0,len(est_T_vicon_rig))]).T

    timestamps_trimmed = []
    idxs = []
    # Now filter by time
    for i in range(0,len(cself.vicon.timestamps)):
        tt = cself.vicon.timestamps[i].to_sec()
        if tt >= tmin and tt <= tmax:
            timestamps_trimmed.append(tt)
            idxs.append(i)
    estX = estX[:,idxs]
    vicX = vicX[:,idxs]

    times = np.array(timestamps_trimmed) - timestamps_trimmed[0]
    xlim = np.array([ times[0], times[-1]])
    axes = []
    if fno is None:
        fno = 1
    # The 3x1 figure errors
    f1 = pl.figure(fno)
    f1.clf()
    pl.title('components')
    errX = estX - vicX;

    pl.plot( times, estX[0,:]*1e3,'r-',lw=3, label='est x')
    pl.plot( times, estX[1,:]*1e3,'g-',lw=3, label='est y')
    pl.plot( times, estX[2,:]*1e3,'b-',lw=3, label='est z')
    pl.plot( times, vicX[0,:]*1e3,'m--',lw=2, label='vic x')
    pl.plot( times, vicX[1,:]*1e3,'y--',lw=2, label='vic y')
    pl.plot( times, vicX[2,:]*1e3,'c--',lw=2, label='vic z')
    pl.legend()
    #pl.plot( times, errX[0,:]*1e3,'m:',times, errX[1,:]*1e3,'y:', errX[2,:]*1e3,'c-',lw=2)
    #pl.plot( times, xx*1e3,'g-',lw=2)
    #pl.plot( times, estX[2.:]*1e3,'b-',lw=2)
    #pl.plot( times, vicX[0,:]*1e3,'m-',lw=2)
    #pl.plot( times, vicX[1.:]*1e3,'y-',lw=2)
    #pl.plot( times, vicX[2.:]*1e3,'c-',lw=2)
    #pl.plot( times, vicX.T*1e3,'c:',lw=2)
    pl.xlabel('time (s)')
    pl.ylabel('translation [mm]')
    pl.xlim(xlim)
    pl.grid('on')

    
def buildOutput(cself, initial, camera):
    cself.camera = camera
    output = Output()
    #output.camera = camera
    found = -1
    for i in range(0,len(initial.cameraTopic)):
        if initial.cameraTopic[i] == camera.topic:
            found = i
            break
    if found < 0:
        raise RuntimeError("Unable to find camera topic {0} in the initial guesses {1}".format(camera.topic, initial.cameraTopic))
    output.cameraIndex = i
    # This is clumsy. Better in the end to use a struct.
    output.T_rig_camera = initial.T_rig_camera[i]
    output.topic = initial.cameraTopic[i]
    output.viconTimeToCameraTime = aslam.cv.Duration(initial.viconTimeToCameraTime[i])
    output.geometry = initial.cameraGeometry[i]
    output.dbName = cself.dbName
    output.bagFileName = cself.bagFileName
    output.timeoffset = cself.calibrated.viconTimeToCameraTime[output.cameraIndex]
    output.missingIntervals = findMissingMeasurementIntervals(cself,cself.calibrated.viconTimeToCameraTime[output.cameraIndex])
    output.measurementIntervals = findMeasurementIntervals(cself,cself.calibrated.viconTimeToCameraTime[output.cameraIndex])
    if hasattr(cself,'optimizationTime'):
        output.optimizationTime = cself.optimizationTime
    output.J = cself.optimizer.J()

    ## Table 2:
    ### The measurement time
    ### The measurements
    ### The predicted measurements
    ### The REs
    imageErrors = []
    for i in range(0,len(camera.reprojectionErrors)):
        mt = camera.reprojectionErrorTimes[i]
        re = camera.reprojectionErrors[i]
        rid = camera.reprojectionErrorIndices[i]
        se = re.evaluateError()
        im_i = []
        im_i.append(mt.toScalar())
        im_i.append(re.getKeypoint())
        im_i.append(re.getProjection())
        im_i.append(re.error())
        im_i.append(se) 
        im_i.append(rid)
        im_i = np.hstack(im_i)
        imageErrors.append(im_i)
    output.imageErrors = np.array(imageErrors)
    output.imageErrorsDescription = "0 time, 1:2 observation, 3:4 projection, 5:6 error, 7 squared error, 8 image index"
    
    output.t_min = output.imageErrors[0,0]
    output.t_max = output.imageErrors[-1,0]
    # What do I need as output?
    # Get the valid vicion timestamps.
    pose_output = []
    pose = []
    timestamps = []
    vic_T_vicon_rig = []
    est_T_vicon_rig = []
    ini_T_vicon_rig = []
    est_spline = cself.designVariables.T_vicon_rig.spline()
    ini_spline = initial.T_vicon_rig
    for i in range(0,len(cself.vicon.timestamps)):
        pose_output_i = []
        pose_i = []
        tt = cself.vicon.timestamps[i].to_sec()
        T_v_r  = cself.vicon.T_vicon_rig[i]
        if tt > output.t_min and tt < output.t_max:
            timestamps.append(tt)
            vic_T_v_r = T_v_r
            est_T_v_r = sm.Transformation( est_spline.transformation(tt) )
            ini_T_v_r = sm.Transformation( ini_spline.transformation(tt) )
            vic_T_vicon_rig.append(vic_T_v_r)
            est_T_vicon_rig.append(est_T_v_r)
            ini_T_vicon_rig.append(ini_T_v_r)
            ini_cerr = sm.R2AxisAngle(np.dot(vic_T_v_r.C().T, ini_T_v_r.C()))
            est_cerr = sm.R2AxisAngle(np.dot(vic_T_v_r.C().T, est_T_v_r.C()))
            ini_terr = vic_T_v_r.t() - ini_T_v_r.t()
            est_terr = vic_T_v_r.t() - est_T_v_r.t()
            ini_aerr = np.linalg.norm(ini_cerr)
            est_aerr = np.linalg.norm(est_cerr)
            ini_derr = np.linalg.norm(ini_terr)
            est_derr = np.linalg.norm(est_terr)
            pose_output_i.append(tt)
            pose_output_i.append(ini_cerr)
            pose_output_i.append(ini_terr)
            pose_output_i.append(ini_aerr)
            pose_output_i.append(ini_derr)
            pose_output_i.append(est_cerr)
            pose_output_i.append(est_terr)
            pose_output_i.append(est_aerr)
            pose_output_i.append(est_derr)
            pose_output_i = np.hstack(pose_output_i)
            pose_output.append(pose_output_i)
            pose_i.append(tt)
            pose_i.append(vic_T_v_r.q())
            pose_i.append(vic_T_v_r.t())
            pose_i.append(ini_T_v_r.q())
            pose_i.append(ini_T_v_r.t())
            pose_i.append(est_T_v_r.q())
            pose_i.append(est_T_v_r.t())
            pose_i = np.hstack(pose_i)
            pose.append(pose_i)

    output.poseErrors = np.array(pose_output)
    output.poseErrorsDescription = "0 time, 1:3 ini_cerr, 4:6 ini_terr, 7 ini_aerr, 8 ini_derr, 9:11 est_cerr, 12:14 est_terr, 15 est_aerr, 16 est_derr"
    output.pose = np.array(pose)
    output.poseDescription = "0:3 vic q, 4:6 vic t, 7:10 ini q, 11:13 ini t, 14:17 est q, 18:20 est t"
    output.vic_T_vicon_rig = vic_T_vicon_rig
    output.est_T_vicon_rig = est_T_vicon_rig
    output.ini_T_vicon_rig = ini_T_vicon_rig

    # One more...Go through the observations for each camera computing the same stuff
    # as poseErrors but only for the initial guesses.
    # What do I need as output?
    # Get the valid vicion timestamps.
    
    # Ugh...this is hard. Pull in the discrete trajectory
    vicdt_T_vicon_rig = aslam.simulation.DiscreteTrajectory()

    for i in range(0,len(vic_T_vicon_rig)):
        tt = cself.vicon.timestamps[i].to_nsec()
        T = vic_T_vicon_rig[i]
        vicdt_T_vicon_rig.addPose( tt, T )

    # Now that I have a discrete trajectory, I can SLERP to query at non-vicon times
    pose_output = []
    pose = []
    timestamps = []
    vic_T_vicon_rig = []
    ini_T_vicon_rig = []
    keys = np.sort(list(camera.observations.keys()))
    T_vicon_target = initial.T_vicon_target
    T_camera_rig = initial.T_rig_camera[output.cameraIndex].inverse().T()
    for i in range(0,len(keys)):
        pose_output_i = []
        pose_i = []
        obs = camera.observations[keys[i]]
        tt = obs.time() + aslam.cv.Duration(output.timeoffset)
        ttnsec = tt.to_nsec()
        ttsec = tt.to_sec()
        if ttsec > output.t_min and ttsec < output.t_max and camera.success[i] and ttnsec >= vicdt_T_vicon_rig.minTime() and ttnsec < vicdt_T_vicon_rig.maxTime():
            T_target_camerak = obs.T_t_c().T()
            T_vicon_camerak = np.dot(T_vicon_target.T(), T_target_camerak)
            T_vicon_rigk = np.dot(T_vicon_camerak, T_camera_rig)

            timestamps.append(ttsec)
            vic_T_v_r = vicdt_T_vicon_rig.T(ttnsec)
            est_T_v_r = sm.Transformation( est_spline.transformation(ttsec) )
            ini_T_v_r = sm.Transformation(T_vicon_rigk)
            vic_T_vicon_rig.append(vic_T_v_r)
            est_T_vicon_rig.append(est_T_v_r)
            ini_T_vicon_rig.append(ini_T_v_r)
            ini_cerr = sm.R2AxisAngle(np.dot(vic_T_v_r.C().T, ini_T_v_r.C()))
            est_cerr = sm.R2AxisAngle(np.dot(vic_T_v_r.C().T, est_T_v_r.C()))
            ini_terr = vic_T_v_r.t() - ini_T_v_r.t()
            est_terr = vic_T_v_r.t() - est_T_v_r.t()
            ini_aerr = np.linalg.norm(ini_cerr)
            est_aerr = np.linalg.norm(est_cerr)
            ini_derr = np.linalg.norm(ini_terr)
            est_derr = np.linalg.norm(est_terr)
            pose_output_i.append(ttsec)
            pose_output_i.append(ini_cerr)
            pose_output_i.append(ini_terr)
            pose_output_i.append(ini_aerr)
            pose_output_i.append(ini_derr)
            pose_output_i.append(est_cerr)
            pose_output_i.append(est_terr)
            pose_output_i.append(est_aerr)
            pose_output_i.append(est_derr)
            pose_output_i = np.hstack(pose_output_i)
            pose_output.append(pose_output_i)
            pose_i.append(tt)
            pose_i.append(vic_T_v_r.q())
            pose_i.append(vic_T_v_r.t())
            pose_i.append(ini_T_v_r.q())
            pose_i.append(ini_T_v_r.t())
            pose_i.append(est_T_v_r.q())
            pose_i.append(est_T_v_r.t())
            pose_i = np.hstack(pose_i)
            pose.append(pose_i)

    output.iniErrors = np.array(pose_output)
    output.iniErrorsDescription = "0 time, 1:3 ini_cerr, 4:6 ini_terr, 7 ini_aerr, 8 ini_derr, 9:11 est_cerr, 12:14 est_terr, 15 est_aerr, 16 est_derr"
    output.iniPose = np.array(pose)
    output.iniPoseDescription = "0:3 vic q, 4:6 vic t, 7:10 ini q, 11:13 ini t, 14:17 est q, 18:20 est t"    

    return output

def plotOutputReScatter(output):
    # output.imageErrors = np.array(imageErrors)
    # output.imageErrorsDescription = "0 time, 1:2 observation, 3:4 projection, 5:6 error, 7 squared error"
    pass


def plotOutputCurves(output, fno=1, plotMeasurementTimes=True, tmin=0.0):
    #output.pose = np.array(pose)
    #output.poseDescription = "0:3 vic q, 4:6 vic t, 7:10 ini q, 11:13 ini t, 14:17 est q, 18:20 est t" 
    times = output.pose[:,0] - tmin
    estX = output.pose[:,18:21].T
    vicX = output.pose[:,4:7].T
    iniX = output.pose[:,11:14].T
    
    f1 = pl.figure(fno)
    f1.clf()
    pl.plot( times, estX[0,:]*1e3,'r-',lw=3, label='est x')
    pl.plot( times, estX[1,:]*1e3,'g-',lw=3, label='est y')
    pl.plot( times, estX[2,:]*1e3,'b-',lw=3, label='est z')
    pl.plot( times, vicX[0,:]*1e3,'m--',lw=2, label='vic x')
    pl.plot( times, vicX[1,:]*1e3,'y--',lw=2, label='vic y')
    pl.plot( times, vicX[2,:]*1e3,'c--',lw=2, label='vic z')
    pl.plot( times, iniX[0,:]*1e3,'m:',lw=2, label='ini x', alpha=0.5)
    pl.plot( times, iniX[1,:]*1e3,'y:',lw=2, label='ini y', alpha=0.5)
    pl.plot( times, iniX[2,:]*1e3,'c:',lw=2, label='ini z', alpha=0.5)
    pl.legend()
    #pl.plot( times, errX[0,:]*1e3,'m:',times, errX[1,:]*1e3,'y:', errX[2,:]*1e3,'c-',lw=2)
    #pl.plot( times, xx*1e3,'g-',lw=2)
    #pl.plot( times, estX[2.:]*1e3,'b-',lw=2)
    #pl.plot( times, vicX[0,:]*1e3,'m-',lw=2)
    #pl.plot( times, vicX[1.:]*1e3,'y-',lw=2)
    #pl.plot( times, vicX[2.:]*1e3,'c-',lw=2)
    #pl.plot( times, vicX.T*1e3,'c:',lw=2)
    pl.xlabel('time (s)')
    pl.ylabel('translation [mm]')
    #pl.xlim(xlim)
    pl.grid('on')

    if plotMeasurementTimes:
        mtimes = output.imageErrors[:,0] - tmin;
        ylim = pl.ylim()
        pl.vlines(mtimes,ylim[0],ylim[1], color=[0.4,0.4,0.45], lw=2, alpha=0.2)

    pass

def plotOutputReVTime(output, fno=1, plotMean=True, tmin=0.0):
    # output.imageErrors = np.array(imageErrors)
    # output.imageErrorsDescription = "0 time, 1:2 observation, 3:4 projection, 5:6 error, 7 squared error, 8 image index"
    mtimes = output.imageErrors[:,0] - tmin
    f1 = pl.figure(fno)
    f1.clf()
    imidxs = np.unique(output.imageErrors[:,8])
    means = []
    meanTimes = []
    for imidx in imidxs:
        terms = output.imageErrors[ output.imageErrors[:,8] == imidx, :]
        terms = terms[terms[:,0].argsort()]
        se = terms[:,7]
        tt = terms[:,0] - tmin
        pl.plot(tt,np.sqrt(se),'bs-')
        meanTimes.append( np.mean(tt) )
        means.append(np.sqrt(np.mean(se)))
    if plotMean:
        pl.plot(meanTimes,means, 'r-', lw=5, alpha=0.5)
    pl.xlabel('time (s)')
    pl.ylabel('error (pixels)')
    pl.grid('on')
                       

def plotOutputError(output, fno=1, plotMeasurementTimes=True, plotInitialization=False, tmin=0.0):
    #output.pose = np.array(pose)
    #output.poseErrorsDescription = "0 time, 1:3 ini_cerr, 4:6 ini_terr, 7 ini_aerr, 8 ini_derr, 9:11 est_cerr, 12:14 est_terr, 15 est_aerr, 16 est_derr"

    #output.poseErrors = np.array(pose_output)
    #output.poseErrorsDescription = "0 time, 1:3 ini_cerr, 4:6 ini_terr, 7 ini_aerr, 8 ini_derr, 9:11 est_cerr, 12:14 est_terr, 15 est_aerr, 16 est_derr"
    times = output.pose[:,0] - tmin
    
    f1 = pl.figure(fno)
    f1.clf()

    pl.plot( times, output.poseErrors[:,16]*1e3, 'b-', label='estimated', lw=2)
    if plotInitialization:
        pl.plot( times, output.poseErrors[:,8]*1e3, 'b:', label='initialized', lw=2)
    pl.xlabel('time (s)')
    pl.ylabel('translation error [mm]')
    pl.grid('on')

    if plotMeasurementTimes:
        mtimes = output.imageErrors[:,0] - tmin;
        ylim = pl.ylim()
        pl.vlines(mtimes,ylim[0],ylim[1], color=[0.4,0.4,0.45], lw=2, alpha=0.2)

def plotOutputErrors(outputs, labels, fno=1, showLegend=True, tmin=0.0):
    #output.pose = np.array(pose)
    #output.poseErrorsDescription = "0 time, 1:3 ini_cerr, 4:6 ini_terr, 7 ini_aerr, 8 ini_derr, 9:11 est_cerr, 12:14 est_terr, 15 est_aerr, 16 est_derr"

    #output.poseErrors = np.array(pose_output)
    #output.poseErrorsDescription = "0 time, 1:3 ini_cerr, 4:6 ini_terr, 7 ini_aerr, 8 ini_derr, 9:11 est_cerr, 12:14 est_terr, 15 est_aerr, 16 est_derr"
    #tmin = np.min( np.array([o.t_min for o in outputs]))
    col = pl.cm.jet(np.array(range(0,len(outputs)),dtype=float)/len(outputs))
    f1 = pl.figure(fno)
    f1.clf()
    
    for i in range(0,len(outputs)):
        output = outputs[i]
        label = labels[i]
        times = output.pose[:,0] - tmin        
        pl.plot( times, output.poseErrors[:,16]*1e3, color=col[i,:], label=label, lw=2)

    pl.xlabel('time (s)')
    pl.ylabel('translation error [mm]')
    pl.grid('on')
    if showLegend:
        pl.legend()
def plotOutputCheckerboard(cself, output, o, rescale=1.0, fno=1):
    camera = cself.cameras[output.cameraIndex]
    imageTable = "{0}_images".format(camera.topic)
    images = cself.db.getTable(imageTable)

    f = pl.figure(fno)
    f.clf()
    I = images[o]
    pl.imshow(I,cmap=pl.cm.gray)
    XL = pl.xlim()
    YL = pl.ylim()
    #imageErrors: 0 time, 1:2 observation, 3:4 projection, 5:6 error, 7 squared error, 8 image index
    # Get the rows corresponding to this image.
    Y = output.imageErrors[ output.imageErrors[:,8] == o,:]
    for i in range(0,Y.shape[0]):
        y = Y[i,1:3]
        haty = Y[i,3:5]
        dy = rescale * (haty-y);
        pl.plot([y[0],y[0]+dy[0]],[y[1],y[1] + dy[1]],'c-', lw=2, alpha=0.7)        
        pl.plot(y[0],y[1],'rx',mew=2)
        pl.plot(haty[0],haty[1],'m+',mew=2)

    pl.xlim(XL)
    pl.ylim(YL)
    pl.title('image {0}, camera {1}'.format(o,output.cameraIndex))
    pl.draw()
    pl.show()

def plotOutputCheckerboardSequence(cself, output, rescale=1.0, fno=1, save=False):
    if save:
        outpath = 'output_checkerboard_sequence'
        if not os.path.exists(outpath):
            os.makedirs(outpath)
    #pl.close('all')
    # Build the points in the world frame
    imidxs = np.unique(output.imageErrors[:,8])
    imidxs.sort()
    imidxs = np.array(imidxs,dtype=int)
    # HACK! Remove observations
    for o in imidxs:
        plotOutputCheckerboard(cself, output, o, rescale=rescale, fno=fno)
        if save:
            f = pl.figure(fno)
            f.savefig(os.path.join(outpath,'image_%06d.png' % o))
