import wpilib
import rev

class Claw:

    def __init__(self, motor_id, _release_speed, _release_change, _intake_speed, _intake_change):

        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.motor = rev.CANSparkMax(motor_id, motor_type)
        self.encoder = self.motor.getEncoder()

        # Just to make sure it's defined.
        self.basePosition = self.encoder.getPosition()

        self.releaseSpeed = _release_speed
        self.releaseChange = _release_change
        self.intakeSpeed = _intake_speed
        self.intakeChange = _intake_change

        # Resets maneuver. Note that this is also reset with off(), which is called in roboty.py: disabledExit()
        self.maneuverComplete = True
        
    # Expel the object by running motors to expel.
    def release(self):
        self.motor.set(self.releaseSpeed)

    # Expel the object by running motors to expel.
    def intake(self):
        self.motor.set(self.intakeSpeed)

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
    
        # Are we expelling game piece and not there yet?
        if (self.maneuverComplete == False) and (self.basePosition < self.targetPosition) and (self.encoder.getPosition() < self.targetPosition):
            self.motor.set(self.releaseSpeed)
            return False

        # Are we grabbing game piece and not there yet?
        elif (self.maneuverComplete == False) and (self.basePosition > self.targetPosition) and (self.encoder.getPosition() > self.targetPosition):
            self.motor.set(self.intakeSpeed)
            return False
        
        # We must be done, so end maneuver.
        else:
            self.motor.set(0)
            self.maneuverComplete = True
            return True
