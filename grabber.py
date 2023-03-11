import wpilib
import rev

class Grabber:

    def __init__(self, rotate_motor_id, bottom_switch_id, top_switch_id, _rotate_speed):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushed
        self.rotate_motor = rev.CANSparkMax(rotate_motor_id, motor_type)
        self.bottom_switch = wpilib.DigitalInput(bottom_switch_id)
        self.top_switch = wpilib.DigitalInput(top_switch_id)
        self.isEngaged = False
        self.rotate_speed = _rotate_speed
        #0 is lowered state, 1 is raised state
        self.state = 0

        #raise rotate motor
    def raise_motor(self, grabber_speed):
        self.state = 1
        self.rotate_motor.set(grabber_speed * 0.7)
        print("RM: top: ", not self.top_switch.get(), " bottom: ", not self.bottom_switch.get())
        if self.top_switch.get() == False:
            self.rotate_motor.set(0)
            return True
        return False
    
    #lower rotate motor
    def lower_motor(self, grabber_speed):
        self.state = 0
        self.rotate_motor.set(grabber_speed * 0.2)
        print("LM: top: ", not self.top_switch.get(), " bottom: ", not self.bottom_switch.get())
        if self.bottom_switch.get() == False:
            self.rotate_motor.set(0)
            return True
        return False

    def motor_off(self):
        self.rotate_motor.set(0)

    #called every loop, used for check if limit switch is activated
    def update(self):
        print("UP: top: ", not self.top_switch.get(), " bottom: ", not self.bottom_switch.get())
        if self.top_switch.get() == False and self.state == 1:
        #if self.state == 1:
            self.rotate_motor.set(0)
            return True
        elif self.bottom_switch.get() == False and self.state == 0:
        #elif self.state == 0:
            self.rotate_motor.set(0)
            return True
        return False

    def atLowerLimit(self):
        return not self.bottom_switch.get()

    def atUpperLimit(self):
        return not self.top_switch.get()