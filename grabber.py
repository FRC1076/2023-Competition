
import rev

class Grabber:

    def __init__(self, motor_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.motor = rev.CANSparkMax(motor_id, motor_type) # elevator up-down

    def grab(self):
        print('Grabber is grabbing!')
        pass

    def release(self):
        print('Grabber is releasing!')
        pass

