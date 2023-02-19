import wpilib

class Cliffsensor:
    
    def __init__(self):
        self.Lsensor = wpilib.Ultrasonic(0,1)
        self.Rsensor = wpilib.Ultrasonic(2,3)
        self.Lsensor.setAutomaticMode(True)
        self.Rsensor.setAutomaticMode(True)

    
    def update(self):
        print(self.Lsensor.getRange())
        print(self.Rsensor.getRange())

        