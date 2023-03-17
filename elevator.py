import rev
import wpilib
from wpilib import DoubleSolenoid
import wpimath.controller
from wpimath.controller import PIDController
import math

class Elevator:
    def __init__(self, right_id, left_id, solenoid_forward_id, solenoid_reverse_id, kP, kI, kD, lower_safety, upper_safety, grabber, limit_switch_id):
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
        self.limit_switch = wpilib.DigitalInput(limit_switch_id)

    #1.00917431193 inches per rotation
    def extend(self, targetSpeed):  # controls length of the elevator 
        #print(self.getEncoderPosition())

        currentPosition = self.getEncoderPosition()
        #if currentPosition >= self.upperSafety:
        #    self.grabber.lower_motor()
        #if currentPosition <= self.lowerSafety:
        #    self.grabber.raise_motor()
        
        #if currentPosition >= self.upperSafety and not self.grabber.atLowerLimit():
        #    self.right_motor.set(0)
        #    self.left_motor.set(0)
        #    return
        
        #if currentPosition <= self.lowerSafety and not self.grabber.atUpperLimit():
        #    self.right_motor.set(0)
        #    self.left_motor.set(0)
        #    return
            
        if targetSpeed > 1:
            targetSpeed = 1
        if targetSpeed < -1:
            targetSpeed = -1
            
        #make sure arm doesn't go past limit
        if self.getEncoderPosition() > 33 and targetSpeed < 0:
            self.right_motor.set(0)
            self.left_motor.set(0)
            return
        if self.getEncoderPosition() < 1 and targetSpeed > 0:
            self.right_motor.set(0)
            self.left_motor.set(0)
            return
        
        self.right_motor.set(-targetSpeed)
        self.left_motor.set(-targetSpeed)

    #automatically move to an elevator extension (position) using a pid controller
    def moveToPos(self, targetPosition):
        extendSpeed = self.pid_controller.calculate(self.getEncoderPosition(), targetPosition)
        print("Elevator: moveToPos: ", self.pid_controller.getSetpoint(), " actual position: ", self.getEncoderPosition())
        if(self.pid_controller.atSetpoint()):
            print("Elevator: At set point", self.getEncoderPosition())
            self.extend(0)
            return True
        else:
            print("Elevator: Moving")
            extendSpeed *= -1 # Elevator motor moves reverse direction.
            self.extend(extendSpeed * 0.1)
            return False

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
        if self.solenoid.get() == DoubleSolenoid.Value.kForward:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
            print("Elevator: Toggle: Set to reverse/up.")
        elif self.solenoid.get() == DoubleSolenoid.Value.kReverse or self.solenoid.get() == DoubleSolenoid.Value.kOff:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
            print("Elevator: Toggle: Set forward/down.")
        else:
            print("Elevator: Toggle: How did we get here?")
        return True
    
    def resetEncoders(self):
        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)
    
    def elevatorReset(self):
        print("Elevator: Reseting elevator")
        return True
        #reset grabber (lift it up) after elevator is all the way down
        if self.limit_switch.get() == True:
            print("Elevator: Found the limit switch")
            self.resetEncoders()
            return self.grabber.grabberReset()
        else:
            self.right_motor.set(-0.1)
            self.left_motor.set(-0.1)
            return False
    
    # only reading the right encoder, assuming that left and right will stay about the same
    def getEncoderPosition(self):
        return self.right_encoder.getPosition()