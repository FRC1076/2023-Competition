import rev

class Elevator:
    def __init__(self, right_id, left_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.right_motor = rev.CANSparkMax(right_id, motor_type)
        self.left_motor = rev.CANSparkMax(left_id, motor_type)
    
    #values might be flipped
    def extend(self):
        self.right_motor.set(0.5)
        self.left_motor.set(-0.5)

    def retract(self):
        self.right_motor.set(-0.5)
        self.left_motor.set(0.5)

class ControlArm:
    def __init__(self, raiser: Elevator, extender: Elevator):
        self.raiser = raiser
        self.extender = extender

