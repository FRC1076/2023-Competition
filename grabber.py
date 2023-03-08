import wpilib
import rev

class Grabber:

    def __init__(self, suction_motor_id, rotate_motor_id, bottom_switch_id, top_switch_id):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushed
        self.suction_motor = rev.CANSparkMax(suction_motor_id, motor_type) # elevator up-down
        self.rotate_motor = rev.CANSparkMax(rotate_motor_id, motor_type)
        self.bottom_switch = wpilib.DigitalInput(bottom_switch_id)
        self.top_switch = wpilib.DigitalInput(top_switch_id)
        self.isEngaged = False
        #0 is lowered state, 1 is raised state
        self.state = 0

    #turn on suction
    def engage(self):
        self.isEngaged = True
        self.execute()
        return True # task is complete

    #turn off suction
    def release(self):
        self.isEngaged = False
        self.execute()
        return True # task is complete
    
    #toggle suction on/off
    def toggle(self):
        self.isEngaged = not self.isEngaged
        self.execute()
        return True # probably not needed?

    #raise rotate motor
    def raise_motor(self):
        self.state = 1
        self.rotate_motor.set(0.5)
        #if self.top_switch.get() == True and self.state == 1:
        #    self.rotate_motor.set(0)
        #    return True
        return False
    
    #lower rotate motor
    def lower_motor(self):
        self.state = 0
        self.rotate_motor.set(-0.5)
        #if self.bottom_switch.get() == True and self.state == 0:
        #    self.rotate_motor.set(0)
        #    return True
        return False

    def motor_off(self):
        self.rotate_motor.set(0)

    #run suction motor
    def execute(self):
        if self.isEngaged:
            self.suction_motor.set(0.2)
        else:
            self.suction_motor.set(0)
    
    #called every loop, used for check if limit switch is activated
    def update(self):
        return True
        print("self.rotate_moter.get()", self.rotate_motor.get())
        #if self.top_switch.get() == True and self.state == 1:
        if self.state == 1:
            self.rotate_motor.set(0)
            return True
        #elif self.bottom_switch.get() == True and self.state == 0:
        elif self.state == 0:
            self.rotate_motor.set(0)
            return True
        return False