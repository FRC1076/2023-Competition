import wpilib
import rev
from logger import Logger

from robotconfig import MODULE_NAMES

DASH_PREFIX = MODULE_NAMES.CLAW

class Claw:

    def __init__(self, motor_id, _cone_default_release_speed, _cone_upper_scoring_height_release_speed, _cone_lower_scoring_height_release_speed, _cube_default_release_speed, _cube_upper_scoring_height_release_speed, _cube_lower_scoring_height_release_speed, _release_change, _intake_speed, _intake_change):
        self.logger = Logger.getLogger()
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.motor = rev.CANSparkMax(motor_id, motor_type)
        self.encoder = self.motor.getEncoder()

        # Just to make sure it's defined and off.
        self.basePosition = self.encoder.getPosition()
        self.motor.set(0)

        self.coneDefaultReleaseSpeed = _cone_default_release_speed
        self.coneUpperReleaseSpeed = _cone_upper_scoring_height_release_speed
        self.coneLowerReleaseSpeed = _cone_lower_scoring_height_release_speed
        self.cubeDefaultReleaseSpeed = _cube_default_release_speed
        self.cubeUpperReleaseSpeed = _cube_upper_scoring_height_release_speed
        self.cubeLowerReleaseSpeed = _cube_lower_scoring_height_release_speed
        
        self.motor.setOpenLoopRampRate(0.0)
        
        self.releaseSpeed = self.coneDefaultReleaseSpeed
        self.releaseChange = _release_change
        self.intakeSpeed = _intake_speed
        self.intakeChange = _intake_change

        # Resets maneuver. Note that this is also reset with off(), which is called in roboty.py: disabledExit()
        self.maneuverComplete = True

    def setConeSpeedDefault(self):
        self.releaseSpeed = self.coneDefaultReleaseSpeed

    def setConeSpeedUpper(self):
        self.releaseSpeed = self.coneUpperReleaseSpeed
            
    def setConeSpeedLower(self):
        self.releaseSpeed = self.coneLowerReleaseSpeed

    def setCubeSpeedDefault(self):
        self.releaseSpeed = self.cubeDefaultReleaseSpeed

    def setCubeSpeedUpper(self):
        self.releaseSpeed = self.cubeUpperReleaseSpeed
            
    def setCubeSpeedLower(self):
        self.releaseSpeed = self.cubeLowerReleaseSpeed

    def getReleaseSpeed(self):
        return self.releaseSpeed
            
    # Expel the object by running motors to expel.
    def release(self):
        self.motor.set(self.releaseSpeed)

    # Expel the object by running motors to expel.
    def intake(self):
        self.motor.set(-self.intakeSpeed)

    # Stop the claw motor.
    def off(self):
        self.motor.set(0)
        self.maneuverComplete = True

    def runAndStop(self, direction):

        # First time in, so figure out where we are and how far to go.
        if self.maneuverComplete == True:
            self.basePosition = self.encoder.getPosition()
            if direction >= 0:
                self.targetPosition = self.basePosition + self.releaseChange
            else:
                self.targetPosition = self.basePosition - self.intakeChange
            self.maneuverComplete = False
            self.log("Claw: RunNStop: FirstTime: basePosition: ", self.basePosition, " self.targetPosition", self.targetPosition)
    
        # Are we expelling game piece and not there yet?
        if (self.maneuverComplete == False) and (self.basePosition < self.targetPosition) and (self.encoder.getPosition() < self.targetPosition):
            self.motor.set(self.releaseSpeed)
            self.log("Claw: RunNStop: Expelling / Not Done: basePosition: ", self.basePosition, " self.targetPosition", self.targetPosition, " getPosition: ", self.encoder.getPosition())
            return False

        # Are we grabbing game piece and not there yet?
        elif (self.maneuverComplete == False) and (self.basePosition > self.targetPosition) and (self.encoder.getPosition() > self.targetPosition):
            self.motor.set(-self.intakeSpeed)
            self.log("Claw: RunNStop: Intake / Not Done: basePosition: ", self.basePosition, " self.targetPosition", self.targetPosition, " getPosition: ", self.encoder.getPosition())
            return False
        
        # We must be done, so end maneuver.
        else:
            self.motor.set(0)
            self.maneuverComplete = True
            return True

    def log(self, *dataToLog):
        self.logger.log(DASH_PREFIX, dataToLog)