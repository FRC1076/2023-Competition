import wpilib
import wpilib.drive
import wpimath.controller
from wpimath.controller import PIDController
import rev


#from logger import Logger

class Elevator:
    def __init__(self, config):
        #self.currentHeight = config["currentHeight"]
        #self.kP = config["kP"]
        #self.kI = config["kI"]
        #self.kD = config["kD"]
        self.shelfHeightA = config["SHELF_HEIGHT_A"] # Lowest shelf
        self.shelfHeightB = config["SHELF_HEIGHT_B"]
        self.shelfHeightC = config["SHELF_HEIGHT_C"]
        self.shelfHeightD = config["SHELF_HEIGHT_D"]
        self.lowerSafety = config['LOWER_SAFETY']
        self.upperSafety = config['UPPER_SAFETY']
        kP = config['kP'] 
        kI = config['kI']
        kD = config['kD']
        
        #self.logger = Logger.getLogger()
        motorType = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.rightMotor = rev.CANSparkMax(config["RIGHT_MOTOR_ID"], motorType) # elevator up-down
        self.leftMotor = rev.CANSparkMax(config["LEFT_MOTOR_ID"], motorType) # elevator up-down
        self.grabberMotor = rev.CANSparkMax(config["GRABBER_MOTOR_ID"], motorType) # elevator up-down

        self.pidController = PIDController(kP, kI, kD)
        self.pidController.setTolerance(0.3, 0.01)

        self.rightEncoder = self.rightMotor.getEncoder() # measure elevator height
        self.leftEncoder = self.leftMotor.getEncoder() # ""
        self.rightEncoder.setPosition(0)
        self.leftEncoder.setPosition(0)

        self.targetHeight = 0

    def extend(self, targetSpeed):  # controls length of the elevator   
        if targetSpeed > 1:
            targetSpeed = 1
        if targetSpeed < -1:
            targetSpeed = -1
        
        #make sure arm doesn't go past limit
        if self.getEncoderPosition() > self.upperSafety and targetSpeed < 0:
            self.rightMotor.set(0)
            self.leftMotor.set(0)
            #return
        if self.getEncoderPosition() < self.lowerSafety and targetSpeed > 0:
            self.rightMotor.set(0)
            self.leftMotor.set(0)
            #return

        # the motors are running backwards, invert targetSpeed.
        #print("Speed:", targetSpeed) # "targetHeight", targetHeight)
        self.rightMotor.set(-targetSpeed) #test it
        self.leftMotor.set(-targetSpeed)

        return
    
    def moveToHeight(self, shelfLabel):
        if shelfLabel == "A":
            self.targetHeight = self.shelfHeightA
            #print(f"shelf-A [{targetHeight}]","Current Height:",self.getEncoderPosition())
        elif shelfLabel == "B":
             self.targetHeight = self.shelfHeightB
             #print(f"shelf-B [{targetHeight}]")
        elif shelfLabel == "C":
             self.targetHeight = self.shelfHeightC
             #print(f"shelf-C [{targetHeight}]")
        elif shelfLabel == "D":
            self.targetHeight = self.shelfHeightD  
            #print(f"shelf-D [{targetHeight}]")

    
    def resetEncoders(self):
        self.leftEncoder.setPosition(0)
        self.rightEncoder.setPosition(0)
        self.targetPosition = self.getEncoderPosition()

    def getEncoderPosition(self):
        return self.rightEncoder.getPosition()
    
    def intake(self, speed):
        #print("intake")
        self.grabberMotor.set(-speed)

    def eject(self, speed):
        #print("eject")
        self.grabberMotor.set(speed)

    #only called when manual movement is used, used to update target height to current height so that elevator won't fall
    def updateTargetHeight(self):
        self.targetHeight = self.getEncoderPosition()
     #def log(self, *dataToLog):
        #self.logger.log(DASH_PREFIX, dataToLog)

    def execute(self):
        extendSpeed = self.pidController.calculate(self.getEncoderPosition(), self.targetHeight) # speed at which elevator has to move according to PID
        #print("encoder position",self.getEncoderPosition(), "targetHeight", self.targetHeight)
        slowedExtendSpeed = extendSpeed # original number: 0.1125 (speed of elevator reduced to become a decimal number)
        #print("Elevator: moveToPos: ", self.pidController.getSetpoint(), " actual position: ", self.getEncoderPosition(),"Extend speed:", slowedExtendSpeed)
        #put a cap on max speed
        maxCap = 0.2
        minCap = -0.1
        if slowedExtendSpeed > maxCap:
            slowedExtendSpeed = maxCap
        if slowedExtendSpeed < minCap:
            slowedExtendSpeed = minCap
        self.extend(-slowedExtendSpeed)