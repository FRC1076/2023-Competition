import rev

class ControlArm:
    def __init__(self, right_id, left_id, rotator_id, intake_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.right_motor = rev.CANSparkMax(right_id, motor_type)
        self.left_motor = rev.CANSparkMax(left_id, motor_type)
        self.rotator_motor = rev.CANSparkMax(rotator_id, motor_type)
        self.intake_motor = rev.CANSparkMax(intake_id, motor_type)
        self.lowered = False

    def extend(self, value):
        self.right_motor.set(value)
        self.left_motor.set(-value)
    
    def loop(self):
        if self.lowered:
            self.intake_motor.set(1)
        else:
            self.intake_motor.set(0)
