
import rev

class Grabber:

    def __init__(self, motor_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.motor = rev.CANSparkMax(motor_id, motor_type) # elevator up-down

    def grab(self):
        self.motor.set(1)
        print('Grabber is grabbing!')

    def release(self):
        self.motor.set(0)
        print('Grabber is releasing!')

