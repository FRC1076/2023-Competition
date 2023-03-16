import wpilib
import rev

class Claw:

    def __init__(self, motor_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.motor = rev.CANSparkMax(motor_id, motor_type)
        self.is_running = False

    #run and stop motor
    def toggle(self):
        if self.is_running == False:
            self.run()
        else:
            self.stop()
        return True
    
    def run(self):
        self.motor.set(1)
        self.is_running = True
        return True

    #Slowly release the game piece from the claw
    def stop(self):
        self.motor.set(-0.3)
        self.is_running = False
        return True