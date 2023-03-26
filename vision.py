# https://docs.limelightvision.io/en/latest/networktables_api.html for NetworkTable values

from networktables import NetworkTables

class Vision:
    def __init__(self, _table, _apriltags, _retroreflective, _min_target_aspect_ration_reflective, _max_target_aspect_ration_reflective, _min_target_aspect_ration_apriltag, _max_target_aspect_ration_apriltag, _shouldUpdatePose):
        self.table = _table
        self.apriltags = _apriltags
        self.retroreflective = _retroreflective
        self.minTargetAspectRatioReflective = _min_target_aspect_ration_reflective
        self.maxTargetAspectRatioReflective = _max_target_aspect_ration_reflective
        self.minTargetAspectRatioAprilTag = _min_target_aspect_ration_apriltag
        self.maxTargetAspectRatioAprilTag = _max_target_aspect_ration_apriltag
        
        self.pipeline = self.retroreflective
        self.table.putNumber('pipeline', self.retroreflective) # default to retro pipeline
        self.updatePose = _shouldUpdatePose

    def shouldUpdatePose(self):
        return self.updatePose
        
    def getPipeline(self):
        self.pipeline = self.table.getNumber('getpipe', 0)
        return self.pipeline

    def setToReflectivePipeline(self):
        self.setPipeline(self.retroreflective)

    def setToAprilTagPipeline(self):
        self.setPipeline(self.apriltags)

    def setPipeline(self, pl : int):
        if 0 <= pl <= 1: # change numbers to reflect min/max pipelines
            self.pipeline = pl
            self.table.putNumber('pipeline', pl)
        else:
            print('Invalid pipeline input: ' + pl)

    def hasTargets(self):
        #print("Vision: ", self.table.getNumber('tv', 0))
        #print("Bot Pose: ", self.table.getNumberArray('botpose', None))
        return bool(self.table.getNumber('tv', 0))

    def canUpdatePose(self):
        #print("Vision: self.pipeline: ", self.pipeline, " self.hasTargets: ", self.hasTargets())
        #print("Vision: getDescription:", self.table.getString('description', 'ABBA'))
        if (self.pipeline == 0 and self.hasTargets()):
            return True
        return False

    def getTargetOffsetX(self):
        if self.hasTargets():
            return self.table.getNumber('tx', 0)
        else:
            print('No vision target.')

    def getTargetOffsetY(self):
        if self.hasTargets():
            return self.table.getNumber('ty', 0)
        else:
            print('No vision target.')

    def getTargetArea(self):
        if self.hasTargets():
            return self.table.getNumber('ta', 0)
        else:
            print('No vision target.')
    
    def getPose(self):
        """
        Returns the robot's calculated field position (x, y, z) in inches relative to the center of the field.
        """
        s = 39.37 # scalar to convert meters to inches
        pose = self.table.getNumberArray('botpose', None) # returns [x, y, z, roll, pitch, yaw]
        print("POSE IS: ", pose)
        if len(pose) != 0:
            return (pose[0] * s, pose[1] * s, pose[2] * s)
        else:
            return (-1, -1, -1)

    def getOrientation(self):
        pose = self.table.getNumberArray('botpose', None) # returns [x, y, z, roll, pitch, yaw]
        print("POSE IS: ", pose)
        if len(pose) != 0:
            return (pose[3], pose[4], pose[5])
        else:
            return (-1, -1, -1)
        
    # get the target size within the frame in pixels
    # can mulitply this by something to get the distance to the target
    # can be compared with the desired distance to drive a PID
    def getTargetSizeReflective(self):
        if not self.hasValidTargetReflective():
            return -1
        return self.table.getNumber('ta', 0.0)
    
    # get the horizontal offset of the target from the 'crosshair'
    # can be compared with the desired offset to drive PID
    def getTargetOffsetHorizontalReflective(self):
        if not self.hasValidTargetReflective():
            return 1000 # more pixels than there are, indicates no valid offset
        return self.table.getNumber('tx', 0.0)

    # determine whether we have one and only target
    # if we don't, we shouldn't use vision 
    def hasValidTargetReflective(self):
        hasTargets = self.hasTargets()
        if not hasTargets:
            return False
        targetHeight = self.table.getNumber('tvert', 100.0)
        targetWidth = self.table.getNumber('thor', 1.0)
        aspectRatio = targetWidth / targetHeight
        return aspectRatio > self.minTargetAspectRatioReflective \
            and aspectRatio < self.maxTargetAspectRatioReflective
    
    # get the target size within the frame in pixels
    # can mulitply this by something to get the distance to the target
    # can be compared with the desired distance to drive a PID
    def getTargetSizeAprilTag(self):
        if not self.hasValidTargetAprilTag():
            return -1
        return self.table.getNumber('ta', 0.0)
    
    # get the horizontal offset of the target from the 'crosshair'
    # can be compared with the desired offset to drive PID
    def getTargetOffsetHorizontalAprilTag(self):
        if not self.hasValidTargetAprilTag():
            return 1000 # more pixels than there are, indicates no valid offset
        return self.table.getNumber('tx', 0.0)

    def hasValidTargetsAprilTags(self):
        hasTargets = self.hasTargets()
        if not hasTargets:
            return False
        targetHeight = self.table.getNumber('tvert', 100.0)
        targetWidth = self.table.getNumber('thor', 1.0)
        aspectRatio = targetWidth / targetHeight
        return aspectRatio > self.minTargetAspectRatioAprilTag \
            and aspectRatio < self.maxTargetAspectRatioAprilTag
    
