import math
import time
import sys

import wpilib
import wpilib.drive
import wpimath.controller
from wpilib import interfaces
import rev
import ctre
from navx import AHRS

from robotconfig import robotconfig
from controller import Controller
from swervedrive import SwerveDrive
from swervemodule import SwerveModule
from swervemodule import ModuleConfig
from swervedrive import BalanceConfig
from swervometer import FieldConfig
from swervometer import RobotPropertyConfig
from swervometer import Swervometer
from tester import Tester
from networktables import NetworkTables

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

# Test Mode
TEST_MODE = False

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):

        self.drivetrain = None
        self.swervometer = None
        self.driver = None
        self.operator = None
        self.tester = None
        self.auton = None

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
            if key == 'SWERVOMETER':
                self.swervometer = self.initSwervometer(config)
            if key == 'DRIVETRAIN':
                self.drivetrain = self.initDrivetrain(config)
            if key == 'AUTON':
                self.auton = self.initAuton(config)

        self.dashboard = NetworkTables.getTable('SmartDashboard')
        self.periods = 0

        if self.drivetrain:
            self.drivetrain.resetGyro()
            self.drivetrain.printGyro()

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

    def initSwervometer(self, config):
        
        if (config['TEAM_IS_RED']):
            team_is_red = True
            team_is_blu = False
        else:
            team_is_red = False
            team_is_blu = True

        if (config['FIELD_START_POSITION'] == 'A'):
            if team_is_red:
                starting_position_x = config['FIELD_RED_A_START_POSITION_X']
                starting_position_y = config['FIELD_RED_A_START_POSITION_Y']
                starting_angle = config['FIELD_RED_A_START_ANGLE']
            else: # team_is_blu
                starting_position_x = config['FIELD_BLU_A_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_A_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_A_START_ANGLE']
        if (config['FIELD_START_POSITION'] == 'B'):
            if team_is_red:
                starting_position_x = config['FIELD_RED_B_START_POSITION_X']
                starting_position_y = config['FIELD_RED_B_START_POSITION_Y']
                starting_angle = config['FIELD_RED_B_START_ANGLE']
            else: # team_is_blu
                starting_position_x = config['FIELD_BLU_B_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_B_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_B_START_ANGLE']
        else: # config['FIELD_START_POSITION'] == 'C'
            if team_is_red:
                starting_position_x = config['FIELD_RED_C_START_POSITION_X']
                starting_position_y = config['FIELD_RED_C_START_POSITION_Y']
                starting_angle = config['FIELD_RED_C_START_ANGLE']
            else: # team_is_blu
                starting_position_x = config['FIELD_BLU_C_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_C_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_C_START_ANGLE']
        
        if config['HAS_BUMPERS_ATTACHED']:
            actual_bumper_dimension_x = config['ROBOT_BUMPER_DIMENSION_X']
            actual_bumper_dimension_y = config['ROBOT_BUMPER_DIMENSION_Y']
        else:
             actual_bumper_dimension_x = 0.0
             actual_bumper_dimension_y = 0.0

        field_cfg = FieldConfig(sd_prefix='Field_Module',
                                origin_x=config['FIELD_ORIGIN_X'],
                                origin_y=config['FIELD_ORIGIN_Y'],
                                start_position_x= starting_position_x,
                                start_position_y= starting_position_y,
                                start_angle= starting_angle,
                                tag1_x=config['FIELD_TAG1_X'],
                                tag1_y=config['FIELD_TAG1_Y'],
                                tag2_x=config['FIELD_TAG2_X'],
                                tag2_y=config['FIELD_TAG2_Y'],
                                tag3_x=config['FIELD_TAG3_X'],
                                tag3_y=config['FIELD_TAG3_Y'],
                                tag4_x=config['FIELD_TAG4_X'],
                                tag4_y=config['FIELD_TAG4_Y'],
                                tag5_x=config['FIELD_TAG5_X'],
                                tag5_y=config['FIELD_TAG5_Y'],
                                tag6_x=config['FIELD_TAG6_X'],
                                tag6_y=config['FIELD_TAG6_Y'],
                                tag7_x=config['FIELD_TAG7_X'],
                                tag7_y=config['FIELD_TAG7_Y'],
                                tag8_x=config['FIELD_TAG8_X'],
                                tag8_y=config['FIELD_TAG8_Y'])
        
        robot_cfg = RobotPropertyConfig(sd_prefix='Robot_Property_Module',
                                is_red_team=team_is_red,
                                frame_dimension_x=config['ROBOT_FRAME_DIMENSION_X'],
                                frame_dimension_y=config['ROBOT_FRAME_DIMENSION_Y'],
                                bumper_dimension_x=actual_bumper_dimension_x,
                                bumper_dimension_y=actual_bumper_dimension_y,
                                com_offset_x=config['ROBOT_COM_OFFSET_X'],
                                com_offset_y=config['ROBOT_COM_OFFSET_Y'],
                                gyro_offset_x=config['ROBOT_GYRO_OFFSET_X'],
                                gyro_offset_y=config['ROBOT_GYRO_OFFSET_Y'],
                                camera_offset_x=config['ROBOT_CAMERA_OFFSET_X'],
                                camera_offset_y=config['ROBOT_CAMERA_OFFSET_Y'])

        swervometer = Swervometer(field_cfg, robot_cfg)

        return swervometer

    def initDrivetrain(self, config):
        
        self.drive_type = config['DRIVETYPE']  # side effect!

        self.rotationCorrection = config['ROTATION_CORRECTION']

        balance_cfg = BalanceConfig(sd_prefix='Balance_Module', balance_pitch_kP=config['BALANCE_PITCH_KP'], balance_pitch_kI=config['BALANCE_PITCH_KI'], balance_pitch_kD=config['BALANCE_PITCH_KD'], balance_yaw_kP=config['BALANCE_YAW_KP'], balance_yaw_kI=config['BALANCE_YAW_KI'], balance_yaw_kD=config['BALANCE_YAW_KD'])

        flModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=190.0, inverted=True, allow_reverse=True, heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        frModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=152.0, inverted=False, allow_reverse=True, heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        rlModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=143.0, inverted=True, allow_reverse=True, heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        rrModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=162.0, inverted=True, allow_reverse=True, heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])

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

        swerve = SwerveDrive(rearLeftModule, frontLeftModule, rearRightModule, frontRightModule, self.swervometer, gyro, balance_cfg)

        return swerve

        #self.testingModule = frontLeftModule

    def initAuton(self, config):
        self.autonScoreExisting = config['SCORE_EXISTING']
        self.autonPickupNew = config['PICKUP_NEW']
        self.scoreNew = config['SCORE_NEW']
        self.balanceBot = config['BALANCE_BOT']
        return True

    def robotPeriodic(self):
        return True

    def teleopInit(self):
        print("teleopInit ran")
        return True

    def teleopPeriodic(self):
        self.teleopDrivetrain()
        return True

    def move(self, x, y, rcw):
        """
        This function is ment to be used by the teleOp.
        :param x: Velocity in x axis [-1, 1]
        :param y: Velocity in y axis [-1, 1]
        :param rcw: Velocity in z axis [-1, 1]
        """

        #print("move: x: ", x, "y: ", y, "rcw: ", rcw)
        # if self.driver.getLeftBumper():
        #     # If the button is pressed, lower the rotate speed.
        #     rcw *= 0.7

        # degrees = (math.atan2(y, x) * 180 / math.pi) + 180

        # self.testingModule.move(rcw, degrees)
        # self.testingModule.execute()

        # print('DRIVE_TARGET = ' + str(rcw) + ', PIVOT_TARGET = ' + str(degrees) + ", ENCODER_TICK = " + str(self.testingModule.get_current_angle()))
        # print('DRIVE_POWER = ' + str(self.testingModule.driveMotor.get()) + ', PIVOT_POWER = ' + str(self.testingModule.rotateMotor.get()))

        self.drivetrain.move(x, y, rcw)

    def teleopDrivetrain(self):
        # if (not self.drivetrain):
        #     return

        driver = self.driver.xboxController
        deadzone = self.driver.deadzone

        speedMulti = 1.0

        self.dashboard.putNumber('ctrl right x', driver.getRightX())
        self.dashboard.putNumber('ctrl right y', driver.getRightY())
        
        
        # Note this is a bad idea in competition, since it's reset automatically in robotInit.
        if (driver.getLeftTriggerAxis() > 0.7 and driver.getRightTriggerAxis() > 0.7):
            self.drivetrain.resetGyro()
            self.drivetrain.printGyro()

        if (driver.getRightBumper()):
            speedMulti = 0.125

        #print("gyro yaw: " + str(self.drivetrain.getGyroAngle()))

        if (driver.getLeftBumper()):
            self.request_wheel_lock = True
        
        if(driver.getAButton()):
            self.drivetrain.balance()
        else:
            self.move(self.deadzoneCorrection(-driver.getRightX(), 0.55 * speedMulti), self.deadzoneCorrection(driver.getRightY(), 0.55 * speedMulti), self.deadzoneCorrection(driver.getLeftX(), 0.2 * speedMulti))
            self.drivetrain.execute()

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
    
    def autonomousInit(self):
        if not self.auton:
            return
        if not self.drivetrain:
            return
        if not self.servometer:
            return

        self.autonTimer = wpilib.Timer()
        self.autonTimer.start()

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
