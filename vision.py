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
        self.pipeline = self.table.getNumber('getpipe')
        return self.pipeline

    def setPipeline(self, pl : int):
        if 0 <= pl <= 1: # change numbers to reflect min/max pipelines
            self.pipeline = pl
            self.table.putNumber('pipeline', pl)
        else:
            print('Invalid pipeline input: ' + pl)

    def hasTargets(self):
        return bool(table.getNumber('tv'))

    def shouldUpdatePosition(self):
        if (self.pipeline == 0 and self.hasTargets):
            return True
        return False

    def getTargetOffsetX(self):
        if self.hasTargets():
            return table.getNumber('tx')
        else:
            print('No vision target.')

    def getTargetOffsetY(self):
        if self.hasTargets():
            return table.getNumber('ty')
        else:
            print('No vision target.')

    def getTargetArea(self):
        if self.hasTargets():
            return table.getNumber('ta')
        else:
            print('No vision target.')
    
    def getPose(self):
        """
        Returns the robot's calculated field position (x, y, z) in inches relative to the center of the field.
        """
        pose = table.getNumberArray('botpose') # returns [x, y, z, roll, pitch, yaw]
        s = 39.37 # scalar to convert meters to inches
        return (pose[0] * s, pose[1] * s, pose[2] * s)

    def getOrientation(self):
        pose = table.getNumberArray('botpose') # returns [x, y, z, roll, pitch, yaw]
        return (pose[3], pose[4], pose[5])