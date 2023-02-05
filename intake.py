import wpilib
import rev

class Intake:
    def __init__(self, motor_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.motor = rev.CANSparkMax(motor_id, motor_type)
        self.running = False
    
    def toggle(self):
        self.running = not self.running
        if self.running:
            self.motor.set(1)
        else:
            self.motor.set(0)