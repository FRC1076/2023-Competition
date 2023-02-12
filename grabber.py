import rev
import wpilib
from wpilib import DoubleSolenoid

class Grabber:
    def __init__(self, right_id, left_id, solenoid_forward_id, solenoid_reverse_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.right_motor = rev.CANSparkMax(right_id, motor_type)
        self.left_motor = rev.CANSparkMax(left_id, motor_type)
        self.solenoid = wpilib.DoubleSolenoid(0, solenoid_forward_id, solenoid_reverse_id)
        #assume retracted
        self.right_motor.getAbsoluteEncoder().setZeroOffset(self.right_motor.getAbsoluteEncoder().getPosition())
        self.left_motor.getAbsoluteEncoder().setZeroOffset(self.left_motor.getAbsoluteEncoder().getPosition())

    #1.00917431193 inches per rotation
    def extend(self, value):
        #make sure arm doesn't go past limit
        if self.right_motor.getAbsoluteEncoder().getPosition() > 35 and value > 0:
            return
        if self.right_motor.getAbsoluteEncoder().getPosition() < 0 and value < 0:
            return
        self.right_motor.set(value)
        self.left_motor.set(-value)

    def toggle(self):
        if self.solenoid.get() == DoubleSolenoid.Value.kForward:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
        elif self.solenoid.get() == DoubleSolenoid.Value.kReverse:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
    
    def getEncoderPosition(self):
        return self.right_motor.getAbsoluteEncoder().getPosition()