import wpilib

class CliffDetector:
    def __init__(self, _leftCliffDetectorPingID, 
    _leftCliffDetectorEchoID, _rightCliffDetectorPingID, 
    _rightCliffDetectorEchoID, _cliffTolerance):
        self.cliffTolerance = _cliffTolerance
        self.Lsensor = wpilib.Ultrasonic(_leftCliffDetectorPingID,_leftCliffDetectorEchoID)
        self.Rsensor = wpilib.Ultrasonic(_rightCliffDetectorPingID, _rightCliffDetectorEchoID)
        self.Lsensor.setAutomaticMode(True)
        self.Rsensor.setAutomaticMode(True)

    def atCliff(self):
        #return -1 # at Left Cliff
        return 0 # not at Cliff
        #return 1 # at Right Cliff
    def update(self):
        print(self.Lsensor.getRange())
        print(self.Rsensor.getRange())

