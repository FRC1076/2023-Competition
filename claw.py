import wpilib
import rev

class Claw:

    def __init__(self, motor_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.motor = rev.CANSparkMax(motor_id, motor_type)

    #run and stop motor
    def toggle(self):
        if self.motor.get() == 0:
            self.run()
        else:
            self.stop()
        return True
    
    def run(self):
        self.motor.set(1)
        return True

    def stop(self):
        self.motor.set(0)
        return True