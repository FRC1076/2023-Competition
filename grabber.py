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
        self.solenoid = wpilib.DoubleSolenoid(1, wpilib.PneumaticsModuleType.REVPH, solenoid_forward_id, solenoid_reverse_id)
        self.pid_controller = PIDController(0.5, 0.00001, 0.00001) #change later
        self.pid_controller.setTolerance(0.00005)

    #1.00917431193 inches per rotation
    def extend(self, value):
        print(self.getEncoderPosition())
        if value > 1:
            value = 1
        if value < -1:
            value = -1
        #make sure arm doesn't go past limit
        if self.getEncoderPosition() > 36 and value < 0:
            self.right_motor.set(0)
            self.left_motor.set(0)
            return
        if self.getEncoderPosition() < 1 and value > 0:
            self.right_motor.set(0)
            self.left_motor.set(0)
            return
        #set to a tenth of the power
        self.right_motor.set(-value * 0.1)
        self.left_motor.set(-value * 0.1)

    #automatically move to a position using a pid controller
    def moveToPos(self, value):
        extend_value = self.pid_controller.calculate(self.getEncoderPosition(), value)
        if(abs(extend_value * 0.3) < 0.01):
            self.extend(0)
            return True
        else:
            print("Moving")
            self.extend(-extend_value * 3) # TO BE RESOLVED: WHICH ONE?
            # self.extend(math.copysign(0.5, self.right_encoder.getPosition() - value))

    def toggle(self):
        if self.solenoid.get() == DoubleSolenoid.Value.kForward:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
        elif self.solenoid.get() == DoubleSolenoid.Value.kReverse or self.solenoid.get() == DoubleSolenoid.Value.kOff:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
    
    def getEncoderPosition(self):
        return self.right_encoder.getPosition()