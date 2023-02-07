import math
import time
import sys

import wpilib
import wpilib.drive
from wpilib import DoubleSolenoid
import wpimath.controller
from wpilib import interfaces
import rev
#import ctre
from navx import AHRS

from robotconfig import robotconfig
from controller import Controller
from swervedrive import SwerveDrive
from swervemodule import SwerveModule
from swervemodule import ModuleConfig
from feeder import Feeder
from tester import Tester
from networktables import NetworkTables
from hooks import Hooks

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

# Test Mode
TEST_MODE = False

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):

        self.drivetrain = None
        self.driver = None
        self.operator = None
        self.feeder = None
        self.tester = None
        self.auton = None
        self.hooks = None
        self.piston = None

        # Even if no drivetrain, defaults to drive phase
        self.phase = "DRIVE_PHASE"

        if TEST_MODE:
            self.config = Tester.getTestConfig()
        else:
            self.config = robotconfig

        print(self.config)
        for key, config in self.config.items():
            if key == 'CONTROLLERS':
                controllers = self.initControllers(config)
                self.driver = controllers[0]
                self.operator = controllers[1]
            if key == 'DRIVETRAIN':
                self.drivetrain = self.initDrivetrain(config)
                print(self.drivetrain)
            if key == 'FEEDER':
                self.feeder = self.initFeeder(config)
            if key == 'AUTON':
                self.auton = self.initAuton(config)
            if key == 'HOOKS':
                self.hooks = self.initHooks(config)
            if key == 'PISTON':
                self.piston = self.initPiston(config)

        self.dashboard = NetworkTables.getTable('SmartDashboard')
        self.periods = 0

        if TEST_MODE:
            self.tester = Tester(self)
            self.tester.initTestTeleop()
            self.tester.testCodePaths()
            

    def initControllers(self, config):
        ctrls = {}
        print(config)
        for ctrlConfig in config.values():
            print(ctrlConfig)
            controller_id = ctrlConfig['ID']
            ctrl = wpilib.XboxController(controller_id)
            dz = ctrlConfig['DEADZONE']
            lta = ctrlConfig['LEFT_TRIGGER_AXIS']
            rta = ctrlConfig['RIGHT_TRIGGER_AXIS']
            ctrls[controller_id] = Controller(ctrl, dz, lta, rta)
        return ctrls


    def initAuton(self, config):
        self.autonHookUpTime = config['HOOK_UP_TIME']
        self.autonDriveForwardTime = config['DRIVE_FORWARD_TIME']
        self.autonHookDownTime = config['HOOK_DOWN_TIME']
        self.autonDriveBackwardTime = config['DRIVE_BACKWARD_TIME']
        self.autonForwardSpeed = config['AUTON_SPEED_FORWARD']
        self.autonBackwardSpeed = config['AUTON_SPEED_BACKWARD']
        return True


    def initDrivetrain(self, config):
        
        self.drive_type = config['DRIVETYPE']  # side effect!

        self.rotationCorrection = config['ROTATION_CORRECTION']

        flModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=190.0, inverted=True, allow_reverse=True)
        frModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=152.0, inverted=False, allow_reverse=True)
        rlModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=143.0, inverted=True, allow_reverse=True)
        rrModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=162.0, inverted=True, allow_reverse=True)

        # flModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=0.0, inverted=True, allow_reverse=True)
        # frModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=0.0, inverted=False, allow_reverse=True)
        # rlModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=0.0, inverted=True, allow_reverse=True)
        # rrModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=0.0, inverted=False, allow_reverse=True)

        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless

        # Drive motors
        flModule_driveMotor = rev.CANSparkMax(config['FRONTLEFT_DRIVEMOTOR'], motor_type)
        frModule_driveMotor = rev.CANSparkMax(config['FRONTRIGHT_DRIVEMOTOR'], motor_type)
        rlModule_driveMotor = rev.CANSparkMax(config['REARLEFT_DRIVEMOTOR'], motor_type)
        rrModule_driveMotor = rev.CANSparkMax(config['REARRIGHT_DRIVEMOTOR'], motor_type)

        # Set ramp rates of drive motors
        #flModule_driveMotor.setClosedLoopRampRate(0.5)
        #frModule_driveMotor.setClosedLoopRampRate(0.5)
        #rlModule_driveMotor.setClosedLoopRampRate(0.5)
        #rrModule_driveMotor.setClosedLoopRampRate(0.5)

        # Rotate motors
        flModule_rotateMotor = rev.CANSparkMax(config['FRONTLEFT_ROTATEMOTOR'], motor_type)
        frModule_rotateMotor = rev.CANSparkMax(config['FRONTRIGHT_ROTATEMOTOR'], motor_type)
        rlModule_rotateMotor = rev.CANSparkMax(config['REARLEFT_ROTATEMOTOR'], motor_type)
        rrModule_rotateMotor = rev.CANSparkMax(config['REARRIGHT_ROTATEMOTOR'], motor_type)

        flModule_encoder = ctre.CANCoder(config['FRONTLEFT_ENCODER'])
        frModule_encoder = ctre.CANCoder(config['FRONTRIGHT_ENCODER'])
        rlModule_encoder = ctre.CANCoder(config['REARLEFT_ENCODER'])
        rrModule_encoder = ctre.CANCoder(config['REARRIGHT_ENCODER'])

        frontLeftModule = SwerveModule(flModule_driveMotor, flModule_rotateMotor, flModule_encoder, flModule_cfg)
        frontRightModule = SwerveModule(frModule_driveMotor, frModule_rotateMotor, frModule_encoder, frModule_cfg)
        rearLeftModule = SwerveModule(rlModule_driveMotor, rlModule_rotateMotor, rlModule_encoder, rlModule_cfg)
        rearRightModule = SwerveModule(rrModule_driveMotor, rrModule_rotateMotor, rrModule_encoder, rrModule_cfg)

        gyro = AHRS.create_spi()

        swerve = SwerveDrive(rearLeftModule, frontLeftModule, rearRightModule, frontRightModule, gyro)

        return swerve

        #self.testingModule = frontLeftModule

    #EXAMPLE
    def initFeeder(self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        feeder = rev.CANSparkMax(config['FEEDER_ID'], motor_type)
        return Feeder(feeder, config['FEEDER_SPEED'])

    def initPiston(self, config):
        pneumatics_module_type = wpilib.PneumaticsModuleType.REVPH

        solenoid = wpilib.DoubleSolenoid(pneumatics_module_type,
                                         config['SOLENOID_FORWARD_ID'],
                                         config['SOLENOID_REVERSE_ID'])
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushed
        self.suction_motor = rev.CANSparkMax(config['MOTOR_ID'], motor_type)
        self.suction_state = False
        self.motor_state = False
        return solenoid

    def robotPeriodic(self):
        return True


    def teleopInit(self):
        print("teleopInit ran")
        self.piston.set(DoubleSolenoid.Value.kOff)
        return True


    def teleopPeriodic(self):
        #self.teleopDrivetrain()
        #self.teleopHooks()
        self.teleopPiston()
        return True

    def move(self, x, y, rcw):
        """
        This function is ment to be used by the teleOp.
        :param x: Velocity in x axis [-1, 1]
        :param y: Velocity in y axis [-1, 1]
        :param rcw: Velocity in z axis [-1, 1]
        """

        # if self.driver.getLeftBumper():
        #     # If the button is pressed, lower the rotate speed.
        #     rcw *= 0.7

        # degrees = (math.atan2(y, x) * 180 / math.pi) + 180

        # self.testingModule.move(rcw, degrees)
        # self.testingModule.execute()

        # print('DRIVE_TARGET = ' + str(rcw) + ', PIVOT_TARGET = ' + str(degrees) + ", ENCODER_TICK = " + str(self.testingModule.get_current_angle()))
        # print('DRIVE_POWER = ' + str(self.testingModule.driveMotor.get()) + ', PIVOT_POWER = ' + str(self.testingModule.rotateMotor.get()))

        self.drivetrain.move(x, y, rcw)
        self.drivetrain.execute()

    def teleopDrivetrain(self):
        # if (not self.drivetrain):
        #     return

        driver = self.driver.xboxController
        deadzone = self.driver.deadzone

        speedMulti = 1.0

        self.dashboard.putNumber('ctrl right x', driver.getRightX())
        self.dashboard.putNumber('ctrl right y', driver.getRightY())
        

        if (driver.getLeftTriggerAxis() > 0.7 and driver.getRightTriggerAxis() > 0.7):
            self.drivetrain.resetGyro()

        if (driver.getRightBumper()):
            speedMulti = 0.125

        #print("gyro yaw: " + str(self.drivetrain.getGyroAngle()))

        if (driver.getLeftBumper()):
            self.request_wheel_lock = True

        self.move(self.deadzoneCorrection(-driver.getRightX(), 0.55 * speedMulti), self.deadzoneCorrection(driver.getRightY(), 0.55 * speedMulti), self.deadzoneCorrection(driver.getLeftX(), 0.2 * speedMulti))

        # Vectoral Button Drive
        #if self.gamempad.getPOV() == 0:
        #    self.drive.set_raw_fwd(-0.35)
        #elif self.gamempad.getPOV() == 180:
        #    self.drive.set_raw_fwd(0.35)
        #elif self.gamempad.getPOV() == 90:
        #    self.drive.set_raw_strafe(0.35)
        #elif self.gamempad.getPOV() == 270:
        #    self.drive.set_raw_strafe(-0.35)
        return

    def initHooks(self, config):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushed

        #Front
        hook1 = rev.CANSparkMax(config['FRONT_HOOK_ID'], motor_type)

        #Back
        hook2 = rev.CANSparkMax(config['BACK_HOOK_ID'], motor_type)

        #Left
        hook3 = rev.CANSparkMax(config['LEFT_HOOK_ID'], motor_type)

        #Right
        hook4 = rev.CANSparkMax(config['RIGHT_HOOK_ID'], motor_type)

        return Hooks([hook1, hook2, hook3, hook4])
    
    def teleopHooks(self):
        operator = self.operator.xboxController

        if operator.getYButtonReleased():
            self.hooks.change_front()

        if operator.getAButtonReleased():
            self.hooks.change_back()

        if operator.getXButtonReleased():
            self.hooks.change_left()

        if operator.getBButtonReleased():
            self.hooks.change_right()

        self.hooks.update()

    def teleopPiston(self):
        driver = self.driver.xboxController

        if driver.getLeftBumperReleased():
            #if vacuuming
            if self.suction_state:
                self.piston.set(DoubleSolenoid.Value.kReverse)
                self.suction_state = False
            else:
                self.piston.set(DoubleSolenoid.Value.kForward)
                self.suction_state = True
        
        if driver.getRightBumperReleased():
            if self.motor_state:
                self.suction_motor.set(0)
                self.motor_state = False
            else:
                self.suction_motor.set(1)
                self.motor_state = True


    def autonomousInit(self):
        if not self.auton:
            return
        if not self.drivetrain:
            return
        self.drivetrain.resetGyro()
        self.autonTimer = wpilib.Timer()
        self.autonTimer.start()
        self.autonHookUp = False
        self.autonHookDown = False

    def autonomousPeriodic(self):
        if not self.auton:
            print("failed self.auton")
            return
        if not self.drivetrain:
            print("failed self.drivetrain")
            return
        
        print("autonomousPeriodic")
        #return
        driver = self.driver.xboxController
        #if driver.getLeftBumper() and driver.getRightBumper():
        
        if self.autonTimer.get() < self.autonHookUpTime:
            print("hook up")
            #if self.autonHookUp == False:
                #self.hooks.change_right()
                #self.autonHookUp == True
            #self.hooks.update()
        elif self.autonHookUpTime <= self.autonTimer.get() < self.autonDriveForwardTime:
            print("move forwards")
            #self.move(0, -self.autonForwardSpeed, 0)
        elif self.autonDriveForwardTime <= self.autonTimer.get() < self.autonHookDownTime:
            print("hook down")
            #if self.autonHookDown == False:
                #self.hooks.change_right()
                #self.autonHookDown = True
            #self.hooks.update()
        elif self.autonHookDownTime <= self.autonTimer.get() < self.autonDriveBackwardTime:
            print("move backwards")
            #self.move(0, self.autonBackwardSpeed, 0)
        #self.hooks.update()

    def deadzoneCorrection(self, val, deadzone):
        """
        Given the deadzone value x, the deadzone both eliminates all
        values between -x and x, and scales the remaining values from
        -1 to 1, to (-1 + x) to (1 - x)
        """
        if abs(val) < deadzone:
            return 0
        elif val < 0:
            x = (abs(val) - deadzone) / (1 - deadzone)
            return -x
        else:
            x = (val - deadzone) / (1 - deadzone)
            return x

    def logResult(self, *result):
        if (TEST_MODE):
            print(result)


if __name__ == "__main__":
    #if sys.argv[1] == 'sim':
    #    TEST_MODE = True
    wpilib.run(MyRobot)
