# https://docs.limelightvision.io/en/latest/networktables_api.html for NetworkTable values

APRILTAGS = 0
RETROREFLECTIVE = 1

class Vision:
    def __init__(self, _table, _shouldUpdatePose):
        self.table = _table
        self.pipeline = APRILTAGS
        self.table.putNumber('pipeline', APRILTAGS) # default to AprilTags pipeline
        self.shouldUpdatePose = _shouldUpdatePose

    def shouldUpdatePose(self):
        return self.shouldUpdatePose
        
    def getPipeline(self):
        self.pipeline = self.table.getNumber('getpipe', 0)
        return self.pipeline

    def setPipeline(self, pl : int):
        if 0 <= pl <= 1: # change numbers to reflect min/max pipelines
            self.pipeline = pl
            self.table.putNumber('pipeline', pl)
        else:
            print('Invalid pipeline input: ' + pl)

    def hasTargets(self):
        print("Vision: ", self.table.getNumber('tv', 0))
        return bool(self.table.getNumber('tv', 0))

    def canUpdatePose(self):
        print("Vision: self.pipeline: ", self.pipeline, " self.hasTargets: ", self.hasTargets())
        print("Vision: getDescription:", self.table.getString('description', 'ABBA'))
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
        pose = self.table.getNumberArray('botpose', None) # returns [x, y, z, roll, pitch, yaw]
        s = 39.37 # scalar to convert meters to inches
        return (pose[0] * s, pose[1] * s, pose[2] * s)

    def getOrientation(self):
        pose = self.table.getNumberArray('botpose', None) # returns [x, y, z, roll, pitch, yaw]
        return (pose[3], pose[4], pose[5])