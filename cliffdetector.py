import wpilib

class CliffDetector:
    def __init__(self, _leftCliffDetector, _rightCliffDetector, _cliffTolerance):
        self.leftCliffDetector = _leftCliffDetector
        self.rightCliffDetector = _rightCliffDetector
        self.cliffTolerance = _cliffTolerance

    def atCliff(self):
        #return -1 # at Left Cliff
        return 0 # not at Cliff
        #return 1 # at Right Cliff

