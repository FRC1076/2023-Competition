import wpilib
import rev
import wpimath.controller
from wpimath.controller import PIDController

from logger import Logger
from robotconfig import MODULE_NAMES

DASH_PREFIX = MODULE_NAMES.GRABBER

class Grabber:

    def __init__(self, rotate_motor_id, _rotate_speed, _rotate_kP, _rotate_kI, _rotate_kD, _max_position, _min_position):
        self.logger = Logger.getLogger()
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.rotate_motor = rev.CANSparkMax(rotate_motor_id, motor_type)
        self.rotate_motor_encoder = self.rotate_motor.getEncoder()
        self.forward_limitSwitch = self.rotate_motor.getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        self.reverse_limitSwitch = self.rotate_motor.getReverseLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        
        self.kP = _rotate_kP
        self.kI = _rotate_kI
        self.kD = _rotate_kD
        self.rotate_pid_controller = PIDController(self.kP, self.kI, self.kD)
        self.rotate_pid_controller.setTolerance(0.1, 0.1)
        self.rotate_motor_encoder.setPosition(10)
        self.targetRotatePosition = self.rotate_motor_encoder.getPosition()
        self.maxRotatePosition = _max_position
        self.minRotatePosition = _min_position
        
        self.rotate_speed = _rotate_speed
        #0 is lowered state, 1 is raised state
        self.state = 0

        self.storeGrabberBypassLimitSwitch = False
        self.grabberHasReset = False

    def resetGrabber(self):
        self.storeGrabberBypassLimitSwitch = False
        self.grabberHasReset = False

    def hasGrabberReset(self):
        #self.grabberHasReset = True
        #return True
        return self.grabberHasReset

    def faultReset(self):
        self.rotate_motor.clearFaults()

    def getEncoderPosition(self):
        return self.rotate_motor_encoder.getPosition()

    def getTargetRotatePosition(self):
        return self.targetRotatePosition

    def setTargetRotatePosition(self, _targetRotatePosition):
        self.targetRotatePosition = _targetRotatePosition

    def constrainTargetPosition(self, _targetRotatePosition):
        self.targetRotatePosition = _targetRotatePosition
        if self.targetRotatePosition < self.minRotatePosition:
            self.targetRotatePosition = self.minRotatePosition
        if self.targetRotatePosition > self.maxRotatePosition:
            self.targetRotatePosition = self.maxRotatePosition
        self.log("Grabber: constrainTargetPosition: target: ", self.targetRotatePosition, " actual: ", self.rotate_motor_encoder.getPosition())
        
    #raise rotate motor
    def raise_motor(self, grabber_speed):
        speed = -grabber_speed
        if self.getForwardLimitSwitch() == True:
            self.log("Grabber: raiseMotor: Turning grabber off.")
            self.rotate_motor.set(0)
            return True
        else:
            self.log("Grabber: actual speed: ", speed, " passed-in grabber_speed: ", grabber_speed)
            self.state = 1
            self.rotate_motor.set(speed * 0.4)
            self.setTargetRotatePosition(self.rotate_motor_encoder.getPosition())
            self.log("Grabber: Raise Motor: top: ", self.getForwardLimitSwitch(), " bottom: ", self.reverse_limitSwitch.get(), " target position: ", self.targetRotatePosition)
            #self.log("Grabber: Final raise motor speed: ", self.rotate_motor.get())
            return False
    
    #lower rotate motor
    def lower_motor(self, grabber_speed):
        speed = -grabber_speed
        if self.reverse_limitSwitch.get() == True:
            self.log("Grabber: lowerMotor: Turning grabber off.")
            self.rotate_motor.set(0)
            return True
        else:
            self.state = 0
            self.rotate_motor.set(speed * 0.4)
            self.setTargetRotatePosition(self.rotate_motor_encoder.getPosition())
            self.log("Grabber: Lower Motor: top: ", self.getForwardLimitSwitch(), " bottom: ", self.reverse_limitSwitch.get(), " target position: ", self.targetRotatePosition)
            return False

    def motor_off(self):
        self.rotate_motor.set(0)

    def move_grabber(self, grabber_speed):
        print("Grabber: Move_grabber", grabber_speed)

        if (grabber_speed > 0):
            self.lower_motor(-grabber_speed)
        elif (grabber_speed < 0):
            self.raise_motor(-grabber_speed)
        else:
            self.update()
    
    #called every loop, used for check if limit switch is activated and PID control
    def update(self):
        self.log("Grabber: Update: top: ", self.getForwardLimitSwitch(), " bottom: ",  self.reverse_limitSwitch.get())
        if self.getForwardLimitSwitch() == True and self.state == 1:
        #if self.state == 1:
            self.log("Grabber: Update: At Top: target position: ", self.targetRotatePosition)
            self.rotate_motor.set(0)
            return True
        elif self.reverse_limitSwitch.get() == True and self.state == 0:
        #elif self.state == 0:
            self.log("Grabber: Update: At Bottom: target position: ", self.targetRotatePosition)
            self.rotate_motor.set(0)
            return True
        else:
            rotate_error = self.rotate_pid_controller.calculate(self.rotate_motor_encoder.getPosition(), self.getTargetRotatePosition())
            #rotate_error = -rotate_error
            #if (self.rotate_motor_encoder.getPosition() < self.minRotatePosition) and rotate_error < 0:
            #    rotate_error = 0
            #if (self.rotate_motor_encoder.getPosition() > self.maxRotatePosition) and rotate_error > 0:
            #    rotate_error = 0
            #rotate_error = -rotate_error
            self.log("Grabber: Update: Fixing encoder error: ", rotate_error, " target position: ", self.getTargetRotatePosition())
            self.rotate_motor.set(rotate_error)
        return False

    def goToPosition(self, target):
        self.setTargetRotatePosition(target)   
        if self.getTargetRotatePosition() < self.minRotatePosition:
            self.log("Grabber: goToPosition: Target position is too low: ", self.getTargetRotatePosition(), " going to ", self.minRotatePosition, " instead.")
            self.setTargetRotatePosition(self.minRotatePosition)
        if self.getTargetRotatePosition() > self.maxRotatePosition:
            self.log("Grabber: goToPosition: Target position is too high: ", self.getTargetRotatePosition(), " going to ", self.maxRotatePosition, " instead.")
            self.setTargetRotatePosition(self.maxRotatePosition)
        rotate_error = self.rotate_pid_controller.calculate(self.rotate_motor_encoder.getPosition(), self.getTargetRotatePosition())
        #if (self.rotate_motor_encoder.getPosition() < self.minRotatePosition) and rotate_error < 0:
        #    rotate_error = 0
        #if (self.rotate_motor_encoder.getPosition() > self.maxRotatePosition) and rotate_error > 0:
        #    rotate_error = 0
        #rotate_error = clamp(rotate_error, 0, 0.2)
        #rotate_error = -rotate_error
        self.log("Grabber: goToPosition: current: ", self.rotate_motor_encoder.getPosition(), "adjustment: ", rotate_error, " target: ", self.getTargetRotatePosition())
        self.rotate_motor.set(rotate_error)
        self.log("Grabber: atSetPoint?", self.rotate_pid_controller.atSetpoint())
        return self.rotate_pid_controller.atSetpoint()
        
    def atLowerLimit(self):
        result = self.reverse_limitSwitch.get()
        self.log("Grabber: atLowerLimit: ", result)
        self.faultReset()
        return result

    def atUpperLimit(self):
        result = self.getForwardLimitSwitch()
        self.log("Grabber: atUpperLimit: ", result)
        self.faultReset()
        return result

    def resetEncoder(self):
        self.log("Grabber: Resetting encoder to zero.")
        self.rotate_motor_encoder.setPosition(0)
    
    def bypassLimitSwitch(self):
        self.log("Grabber: Bypassing limit switch reset.")
        self.storeGrabberBypassLimitSwitch = True
        return
    
    def getForwardLimitSwitch(self):
        return not self.forward_limitSwitch.get() # Hack because it's wired backwards.

    #move grabber to the up position and reset encoders for the grabber (top position is encoder position 0)
    def grabberReset(self):
        if self.atUpperLimit() or self.storeGrabberBypassLimitSwitch:
            self.rotate_motor.set(0)
            self.log("Grabber: grabberReset: completed task")
            self.resetEncoder()
            self.grabberHasReset = True
            self.setTargetRotatePosition(self.rotate_motor_encoder.getPosition())
            self.log("Grabber: grabberReset: Target: ", self.getTargetRotatePosition())
            return True
        else:
            self.log("Grabber: grabberReset: raising motor")
            #self.raise_motor(1.0) #goes at speed of 0.15 * 0.7 = 0.105
            self.rotate_motor.set(-0.4)
            self.log("Grabber: grabberReset: speed: ", self.rotate_motor.get())
            self.grabberHasReset = False
            return False

    def clamp(num, min_value, max_value):
        if num >= 0:
            return max(min(num, max_value), min_value)
        else:
            neg_min = -min_value
            neg_max = -max_value
            return max(min(num, neg_min), neg_max)

    def log(self, *dataToLog):
        self.logger.log(DASH_PREFIX, dataToLog)
