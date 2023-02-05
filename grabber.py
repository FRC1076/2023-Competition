import rev
import wpilib
from wpilib import DoubleSolenoid

class Grabber:
    def __init__(self, right_id, left_id, solenoid_forward_id, solenoid_reverse_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.right_motor = rev.CANSparkMax(right_id, motor_type)
        self.left_motor = rev.CANSparkMax(left_id, motor_type)
        self.solenoid = wpilib.DoubleSolenoid(0, solenoid_forward_id, solenoid_reverse_id)

    def extend(self, value):
        self.right_motor.set(value)
        self.left_motor.set(-value)

    def toggle(self):
        if self.solenoid.get() == DoubleSolenoid.Value.kForward:
            self.solenoid.set(DoubleSolenoid.Value.kOff)
        if self.solenoid.get() == DoubleSolenoid.Value.kOff:
            self.solenoid.set(DoubleSolenoid.Value.kForward)