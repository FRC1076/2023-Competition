
import wpilib
from wpilib import DoubleSolenoid

SOLENOID_OPEN = DoubleSolenoid.Value.kForward
SOLENOID_CLOSED = DoubleSolenoid.Value.kReverse
SOLENOID_OFF = DoubleSolenoid.Value.kOff

class Claw:

    def __init__(self, solenoid_forward_id, solenoid_reverse_id):
        
        self.solenoid_forward_id = solenoid_forward_id
        self.solenoid_reverse_id = solenoid_reverse_id

        self.solenoid = wpilib.DoubleSolenoid(1, 
            wpilib.PneumaticsModuleType.REVPH, 
            solenoid_forward_id, 
            solenoid_reverse_id)
        
    # open and close
    def toggle(self):
        if self.solenoid.get() == SOLENOID_CLOSED:
            self.open()
        elif self.solenoid.get() == SOLENOID_OPEN or self.solenoid.get() == SOLENOID_OFF:
            self.close()
        else:
            print("Elevator: Toggle: How did we get here?")
        return True
    
    def open(self):
        self.solenoid.set(SOLENOID_OPEN)

    def close(self):
        self.solenoid.set(SOLENOID_CLOSED)