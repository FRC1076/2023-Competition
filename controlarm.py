import rev
import wpilib
from wpilib import DoubleSolenoid

class ControlArm:
    def __init__(self, right_id, left_id, intake_top_id, intake_bottom_id, solenoid_forward_id, solenoid_reverse_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.right_motor = rev.CANSparkMax(right_id, motor_type)
        self.left_motor = rev.CANSparkMax(left_id, motor_type)
        self.intake_top_motor = rev.CANSparkMax(intake_top_id, motor_type)
        self.intake_bottom_motor = rev.CANSparkMax(intake_bottom_id, motor_type)
        self.intake = False
        self.solenoid = wpilib.DoubleSolenoid(0, solenoid_forward_id, solenoid_reverse_id)

    def extend(self, value):
        self.right_motor.set(value)
        self.left_motor.set(-value)

    def toggle_arm(self):
        if self.solenoid.get() == DoubleSolenoid.Value.kForward:
            self.solenoid.set(DoubleSolenoid.Value.kOff)
        if self.solenoid.get() == DoubleSolenoid.Value.kOff:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
    
    def toggle_intake(self):
        self.intake = not self.intake
        if self.intake:
            self.intake_top_motor.set(1)
            self.intake_bottom_motor.set(-1)
        else:
            self.intake_top_motor.set(0)
            self.intake_bottom_motor.set(0)