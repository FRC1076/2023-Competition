import rev
import wpilib
from wpilib import DoubleSolenoid
import wpimath.controller
from wpimath.controller import PIDController
import math
from logger import Logger

class Elevator:
    def __init__(self, right_id, left_id, solenoid_forward_id, solenoid_reverse_id, kP, kI, kD, lower_safety, upper_safety, grabber, left_limit_switch_id, right_limit_switch_id):
        self.logger = Logger.getLogger()
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.right_motor = rev.CANSparkMax(right_id, motor_type) # elevator up-down
        self.left_motor = rev.CANSparkMax(left_id, motor_type) # elevator up-down
        self.right_encoder = self.right_motor.getEncoder() # measure elevator height
        self.left_encoder = self.left_motor.getEncoder() # ""
        self.right_encoder.setPosition(0)
        self.left_encoder.setPosition(0)
        self.solenoid = wpilib.DoubleSolenoid(1, # controls the "lean" of the elevator
            wpilib.PneumaticsModuleType.REVPH, 
            solenoid_forward_id, 
            solenoid_reverse_id)
        self.pid_controller = PIDController(kP, kI, kD)
        self.pid_controller.setTolerance(0.6, 0.6)
        self.grabber = grabber
        self.right_motor.setOpenLoopRampRate(0.50)
        self.left_motor.setOpenLoopRampRate(0.50)
        self.upperSafety = upper_safety
        self.lowerSafety = lower_safety
        self.left_limit_switch = wpilib.DigitalInput(left_limit_switch_id)
        self.right_limit_switch = wpilib.DigitalInput(right_limit_switch_id)
        self.targetPosition = self.getEncoderPosition()

    def getTargetPosition(self):
        return self.targetPosition

    #1.00917431193 inches per rotation
    def extend(self, targetSpeed):  # controls length of the elevator 
            
        if targetSpeed > 1:
            targetSpeed = 1
        if targetSpeed < -1:
            targetSpeed = -1
            
        #make sure arm doesn't go past limit
        if self.getEncoderPosition() > self.upperSafety and targetSpeed < 0:
            self.right_motor.set(0)
            self.left_motor.set(0)
            return
        if self.getEncoderPosition() < self.lowerSafety and targetSpeed > 0:
            self.right_motor.set(0)
            self.left_motor.set(0)
            return
        
        self.right_motor.set(-targetSpeed)
        self.left_motor.set(-targetSpeed)

    # Move elevator and reset target to where you end up.
    def move(self, targetSpeed):
        self.extend(targetSpeed)
        self.targetPosition = self.getEncoderPosition()

    #automatically move to an elevator extension (position) using a pid controller
    def moveToPos(self, _targetPosition):
        
        self.targetPosition = _targetPosition
        extendSpeed = self.pid_controller.calculate(self.getEncoderPosition(), self.targetPosition)
        self.log("Elevator: moveToPos: ", self.pid_controller.getSetpoint(), " actual position: ", self.getEncoderPosition())
        if(self.pid_controller.atSetpoint()):
            self.log("Elevator: At set point", self.getEncoderPosition())
            self.extend(0)
            return True
        else:
            self.log("Elevator: Moving")
            extendSpeed *= -1 # Elevator motor moves reverse direction.
            self.extend(extendSpeed * 0.1)
            return False

    def update(self):
        self.moveToPos(self.targetPosition)
    
    def isElevatorDown(self):
        if self.solenoid.get() == DoubleSolenoid.Value.kForward or self.solenoid.get() == DoubleSolenoid.Value.kOff:
            return True
        return False

    def isElevatorUp(self):
        if self.solenoid.get() == DoubleSolenoid.Value.kReverse:
            return True
        return False

    def elevatorUp(self):
        self.solenoid.set(DoubleSolenoid.Value.kReverse)
        return True

    def elevatorDown(self):
        self.solenoid.set(DoubleSolenoid.Value.kForward)
        return True

    # contols the "lean" of the elevator
    def toggle(self):
        self.log("Elevator: In toggle().")
        if self.solenoid.get() == DoubleSolenoid.Value.kForward:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
            self.log("Elevator: Toggle: Set to reverse/up.")
        elif self.solenoid.get() == DoubleSolenoid.Value.kReverse or self.solenoid.get() == DoubleSolenoid.Value.kOff:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
            self.log("Elevator: Toggle: Set forward/down.")
        else:
            self.log("Elevator: Toggle: How did we get here?")
        return True
    
    def resetEncoders(self):
        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)
        self.targetPosition = self.getEncoderPosition()

    def bypassLimitSwitch(self):
        self.log("Elevator: Bypassing limit switch reset.")
        self.resetEncoders()
    
    def elevatorReset(self):
        self.log("Elevator: Reseting elevator")
        
        #reset grabber (lift it up) after elevator is all the way down
        if self.left_limit_switch.get() == True or self.right_limit_switch.get() == True:
            self.log("Elevator: Found the limit switch")
            self.resetEncoders()
            return True
        else:
            self.right_motor.set(-0.1)
            self.left_motor.set(-0.1)
            return False
    
    # only reading the right encoder, assuming that left and right will stay about the same
    def getEncoderPosition(self):
        return self.right_encoder.getPosition()

    def log(self, *dataToLog):
        self.logger.log(dataToLog)