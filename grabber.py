import wpilib
import rev

class Grabber:

    def __init__(self, rotate_motor_id, bottom_switch_id, top_switch_id, _rotate_speed):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.rotate_motor = rev.CANSparkMax(rotate_motor_id, motor_type)
        self.rotate_motor_encoder = self.rotate_motor.getEncoder()
        self.forward_limitSwitch = self.rotate_motor.getForwardLimitSwitch()
        self.reverse_limitSwitch = self.rotate_motor.getReverseLimitSwitch()
        self.bottom_switch = wpilib.DigitalInput(bottom_switch_id)
        self.top_switch = wpilib.DigitalInput(top_switch_id)
        self.rotate_speed = _rotate_speed
        #0 is lowered state, 1 is raised state
        self.state = 0
        self.bypassLimitSwitch = False

    def faultReset(self)
        self.rotate_motor.clearFaults()

    #raise rotate motor
    def raise_motor(self, grabber_speed):
        print("Grabber: grabber_speed: ", grabber_speed)
        self.state = 1
        self.rotate_motor.set(grabber_speed * 0.7)
        print("RM: top: ", self.top_switch.get(), " bottom: ", self.bottom_switch.get())
        if self.top_switch.get() == False:
            print("Grabber: Turning grabber off.")
            self.rotate_motor.set(0)
            return True
        return False
    
    #lower rotate motor
    def lower_motor(self, grabber_speed):
        self.state = 0
        self.rotate_motor.set(grabber_speed * 0.2)
        print("LM: top: ", self.top_switch.get(), " bottom: ", self.bottom_switch.get())
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
        result = self.bottom_switch.getFault(kHardLimitRev)
        print("atLowerLimit: ", result)
        self.faultReset()
        return result

    def atUpperLimit(self):
        result = self.bottom_switch.getFault(kHardLimitFwd)
        print("atUpperLimit: ", result)
        self.faultReset()
        return result

    def resetEncoder(self):
        self.rotate_motor_encoder.setPosition(0)
    
    def bypassLimitSwitch(self):
        print("Grabber: Bypassing limit switch reset.")
        self.bypassLimitSwitch = True
    
    #move grabber to the up position and reset encoders for the grabber (top position is encoder position 0)
    def grabberReset(self):
        if self.atUpperLimit() or self.bypassLimitSwitch == True
            self.resetEncoder()
            return True
        else:
            self.raise_motor(0.15) #goes at speed of 0.15 * 0.7 = 0.105
            return False
