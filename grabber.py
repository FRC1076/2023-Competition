import rev
import wpilib
from wpilib import DoubleSolenoid
import wpimath.controller
from wpimath.controller import PIDController
import math

class Grabber:
    def __init__(self, right_id, left_id, solenoid_forward_id, solenoid_reverse_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.right_motor = rev.CANSparkMax(right_id, motor_type)
        self.left_motor = rev.CANSparkMax(left_id, motor_type)
        self.right_encoder = self.right_motor.getEncoder()
        self.left_encoder = self.left_motor.getEncoder()
        #self.right_encoder.setPosition(0)
        #self.left_encoder.setPosition(0)
        self.solenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, solenoid_forward_id, solenoid_reverse_id)
        self.pid_controller = PIDController(0.5, 0.00001, 0.00001) #change later
        #self.pid_controller.setTolerance(0.005)
        #assume retracted
        #self.right_motor.getEncoder().setZeroOffset(self.right_motor.getEncoder().getPosition())
        #self.left_motor.getEncoder().setZeroOffset(self.left_motor.getEncoder().getPosition())

    #1.00917431193 inches per rotation
    def extend(self, value):
        print("Value:", value)
        if value > 1:
            value = 1
        if value < -1:
            value = -1
        #make sure arm doesn't go past limit
        if self.right_encoder.getPosition() > 36 and value < 0:
            self.right_motor.set(0)
            self.left_motor.set(0)
            return
        if self.right_encoder.getPosition() < 1 and value > 0:
            self.right_motor.set(0)
            self.left_motor.set(0)
            return
        self.right_motor.set(-value * 0.1)
        self.left_motor.set(-value * 0.1)

    def moveToPos(self, value):
        extend_value = self.pid_controller.calculate(self.right_encoder.getPosition(), value)
        if(self.pid_controller.atSetpoint()):
            print("Reached")
            self.extend(0)
            return True
        else:
            print("Moving")
            self.extend(-extend_value * 3)
            return False

    def toggle(self):
        if self.solenoid.get() == DoubleSolenoid.Value.kForward:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
        elif self.solenoid.get() == DoubleSolenoid.Value.kReverse or self.solenoid.get() == DoubleSolenoid.Value.kOff:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
    
    def getEncoderPosition(self):
        return self.right_encoder.getPosition()