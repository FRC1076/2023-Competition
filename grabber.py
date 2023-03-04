
import rev

class Grabber:

    def __init__(self, motor_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.motor = rev.CANSparkMax(motor_id, motor_type) # elevator up-down
        self.isEngaged = False

    def engage(self):
        self.isEngaged = True
        self.execute()
        return True # task is complete

    def release(self):
        self.isEngaged = False
        self.execute()
        return True # task is complete
    
    def toggle(self):
        self.isEngaged = not self.isEngaged
        self.execute()
        return True # probably not needed?

    def execute(self):
        if self.isEngaged:
            self.motor.set(1)
        else:
            self.motor.set(0)
