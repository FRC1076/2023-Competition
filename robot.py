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
from swervedrive import TargetConfig
from swervometer import FieldConfig
from swervometer import RobotPropertyConfig
from swervometer import Swervometer
from tester import Tester
from networktables import NetworkTables

from grabber import Grabber
from vision import Vision
from intake import Intake

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
        self.vision = None
        self.grabber = None

        # Even if no drivetrain, defaults to drive phase
        self.phase = "DRIVE_PHASE"

        if TEST_MODE:
            self.config = Tester.getTestConfig()
        else:
            self.config = robotconfig

        self.dashboard = NetworkTables.getTable('SmartDashboard')

        print(self.config)
        for key, config in self.config.items():
            if key == 'CONTROLLERS':
                controllers = self.initControllers(config)
                self.driver = controllers[0]
                self.operator = controllers[1]
            if key == 'DRIVETRAIN':
                self.drivetrain = self.initDrivetrain(config)
            if key == 'AUTON':
                self.auton = self.initAuton(config
            if key == 'VISION':
                self.vision = self.initVision(config)
            if key == 'SWERVOMETER':
                self.swervometer = self.initSwervometer(config)
            if key == 'GRABBER':
                self.grabber = self.initGrabber(config)
            if key == 'INTAKE':
                self.intake = self.initIntake(config)

        self.periods = 0

        if self.drivetrain:
            self.drivetrain.resetGyro()
            self.drivetrain.printGyro()

        if TEST_MODE:
            self.tester = Tester(self)
            self.tester.initTestTeleop()
            self.tester.testCodePaths()

    def disabledExit(self):
        print("no longer disabled")
        self.drivetrain.reset()

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
        print("initSwervometer ran")
        
        if (config['TEAM_IS_RED']):
            team_is_red = True
            team_is_blu = False
        else:
            team_is_red = False
            team_is_blu = True

        self.dashboard.putBoolean('Team is Red', team_is_red)

        print("FIELD_START_POSITION:", config['FIELD_START_POSITION'])

        if (config['FIELD_START_POSITION'] == 'A'):
            self.dashboard.putString('Field Start Position', 'A')
            if team_is_red:
                starting_position_x = config['FIELD_RED_A_START_POSITION_X']
                starting_position_y = config['FIELD_RED_A_START_POSITION_Y']
                starting_angle = config['FIELD_RED_A_START_ANGLE']
            else: # team_is_blu
                starting_position_x = config['FIELD_BLU_A_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_A_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_A_START_ANGLE']
        elif (config['FIELD_START_POSITION'] == 'B'):
            self.dashboard.putString('Field Start Position', 'B')
            if team_is_red:
                starting_position_x = config['FIELD_RED_B_START_POSITION_X']
                starting_position_y = config['FIELD_RED_B_START_POSITION_Y']
                starting_angle = config['FIELD_RED_B_START_ANGLE']
            else: # team_is_blu
                starting_position_x = config['FIELD_BLU_B_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_B_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_B_START_ANGLE']
        else: # config['FIELD_START_POSITION'] == 'C'
            self.dashboard.putString('Field Start Position', 'C')
            if team_is_red:
                starting_position_x = config['FIELD_RED_C_START_POSITION_X']
                starting_position_y = config['FIELD_RED_C_START_POSITION_Y']
                starting_angle = config['FIELD_RED_C_START_ANGLE']
            else: # team_is_blu
                starting_position_x = config['FIELD_BLU_C_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_C_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_C_START_ANGLE']
        
        bumpers_attached = config['HAS_BUMPERS_ATTACHED']
        if bumpers_attached:
            actual_bumper_dimension_x = config['ROBOT_BUMPER_DIMENSION_X']
            actual_bumper_dimension_y = config['ROBOT_BUMPER_DIMENSION_Y']
        else:
             actual_bumper_dimension_x = 0.0
             actual_bumper_dimension_y = 0.0

        self.dashboard.putBoolean('Has Bumpers Attached', bumpers_attached)

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
                                camera_offset_y=config['ROBOT_CAMERA_OFFSET_Y'],
                                swerve_module_offset_x=config['ROBOT_SWERVE_MODULE_OFFSET_X'],
                                swerve_module_offset_y=config['ROBOT_SWERVE_MODULE_OFFSET_Y'])

        swervometer = Swervometer(field_cfg, robot_cfg)

        return swervometer
        
    def initGrabber(self, config):
        grabber = Grabber(config['RIGHT_I'], config['LEFT_ID'], config['SOLENOID_FORWARD_ID'], config['SOLENOID_REVERSE_ID'])
        return grabber

    def initAuton(self, config):
        self.autonHookUpTime = config['HOOK_UP_TIME']
        self.autonDriveForwardTime = config['DRIVE_FORWARD_TIME']
        self.autonHookDownTime = config['HOOK_DOWN_TIME']
        self.autonDriveBackwardTime = config['DRIVE_BACKWARD_TIME']
        self.autonForwardSpeed = config['AUTON_SPEED_FORWARD']
        self.autonBackwardSpeed = config['AUTON_SPEED_BACKWARD']
        self.autonScoreExisting = config['SCORE_EXISTING']
        self.autonPickupNew = config['PICKUP_NEW']
        self.scoreNew = config['SCORE_NEW']
        self.balanceBot = config['BALANCE_BOT']
        return True


    def initDrivetrain(self, config):
        print("initDrivetrain ran")
        self.drive_type = config['DRIVETYPE']  # side effect!

        self.rotationCorrection = config['ROTATION_CORRECTION']

        balance_cfg = BalanceConfig(sd_prefix='Balance_Module', balance_pitch_kP=config['BALANCE_PITCH_KP'], balance_pitch_kI=config['BALANCE_PITCH_KI'], balance_pitch_kD=config['BALANCE_PITCH_KD'], balance_yaw_kP=config['BALANCE_YAW_KP'], balance_yaw_kI=config['BALANCE_YAW_KI'], balance_yaw_kD=config['BALANCE_YAW_KD'])
        target_cfg = TargetConfig(sd_prefix='Target_Module', target_kP=config['TARGET_KP'], target_kI=config['TARGET_KI'], target_kD=config['TARGET_KD'])

        flModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=190.5, inverted=True, allow_reverse=True, position_conversion=config['ROBOT_INCHES_PER_ROTATION'], heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        frModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=153.3, inverted=False, allow_reverse=True, position_conversion=config['ROBOT_INCHES_PER_ROTATION'], heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        rlModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=143.8, inverted=True, allow_reverse=True, position_conversion=config['ROBOT_INCHES_PER_ROTATION'], heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        rrModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=161.5, inverted=True, allow_reverse=True, position_conversion=config['ROBOT_INCHES_PER_ROTATION'], heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless

        # Drive motors
        flModule_driveMotor = rev.CANSparkMax(config['FRONTLEFT_DRIVEMOTOR'], motor_type)
        flModule_driveMotor_encoder = flModule_driveMotor.getEncoder()
        frModule_driveMotor = rev.CANSparkMax(config['FRONTRIGHT_DRIVEMOTOR'], motor_type)
        frModule_driveMotor_encoder = frModule_driveMotor.getEncoder()
        rlModule_driveMotor = rev.CANSparkMax(config['REARLEFT_DRIVEMOTOR'], motor_type)
        rlModule_driveMotor_encoder = rlModule_driveMotor.getEncoder()
        rrModule_driveMotor = rev.CANSparkMax(config['REARRIGHT_DRIVEMOTOR'], motor_type)
        rrModule_driveMotor_encoder = rrModule_driveMotor.getEncoder()

        flModule_rotateMotor_encoder = ctre.CANCoder(config['FRONTLEFT_ENCODER'])
        frModule_rotateMotor_encoder = ctre.CANCoder(config['FRONTRIGHT_ENCODER'])
        rlModule_rotateMotor_encoder = ctre.CANCoder(config['REARLEFT_ENCODER'])
        rrModule_rotateMotor_encoder = ctre.CANCoder(config['REARRIGHT_ENCODER'])

        frontLeftModule = SwerveModule(flModule_driveMotor, flModule_driveMotor_encoder, flModule_rotateMotor, flModule_rotateMotor_encoder, flModule_cfg)
        frontRightModule = SwerveModule(frModule_driveMotor, frModule_driveMotor_encoder, frModule_rotateMotor, frModule_rotateMotor_encoder, frModule_cfg)
        rearLeftModule = SwerveModule(rlModule_driveMotor, rlModule_driveMotor_encoder, rlModule_rotateMotor, rlModule_rotateMotor_encoder, rlModule_cfg)
        rearRightModule = SwerveModule(rrModule_driveMotor, rrModule_driveMotor_encoder, rrModule_rotateMotor, rrModule_rotateMotor_encoder, rrModule_cfg)

        #gyro = AHRS.create_spi()
        gyro = AHRS.create_spi(wpilib._wpilib.SPI.Port.kMXP, 500000, 50) # https://www.chiefdelphi.com/t/navx2-disconnecting-reconnecting-intermittently-not-browning-out/425487/36
        
        swerve = SwerveDrive(rearLeftModule, frontLeftModule, rearRightModule, frontRightModule, self.swervometer, gyro, balance_cfg, target_cfg)

        return swerve

    def initAuton(self, config):
        self.autonScoreExisting = config['SCORE_EXISTING']
        self.autonPickupNew = config['PICKUP_NEW']
        self.autonScoreNew = config['SCORE_NEW']
        self.autonBalanceRobot = config['BALANCE_BOT']
        self.dashboard.putNumber('Auton Score Existing Element', self.autonScoreExisting)
        self.dashboard.putNumber('Auton Pickup New Element', self.autonPickupNew)
        self.dashboard.putNumber('Auton Score New Element', self.autonScoreNew)
        self.dashboard.putNumber('Auton Balance Robot', self.autonBalanceRobot)

        return True

    def initVision(self, config):
        vision = Vision(NetworkTables.getTable('limelight'))

        return vision

    def robotPeriodic(self):
        return True

    def teleopInit(self):
        print("teleopInit ran")
        return True

    def teleopPeriodic(self):
        self.teleopDrivetrain()
        self.teleopGrabber()
        self.teleopIntake()
        return True

    def move(self, x, y, rcw):
        """
        This function is ment to be used by the teleOp.
        :param x: Velocity in x axis [-1, 1]
        :param y: Velocity in y axis [-1, 1]
        :param rcw: Velocity in z axis [-1, 1]
        """

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
        
        # Note this is a bad idea in competition, since it's reset automatically in robotInit.
        if (driver.getLeftTriggerAxis() > 0.7 and driver.getRightTriggerAxis() > 0.7):
            self.drivetrain.resetGyro()
            self.drivetrain.printGyro()

        if (driver.getRightBumper()):
            speedMulti = 0.125

        #print("gyro yaw: " + str(self.drivetrain.getGyroAngle()))

        if (driver.getLeftBumper()):
            self.drivetrain.setWheelLock(True)
        else:
            self.drivetrain.setWheelLock(False)
        
        if(driver.getAButton()):
            self.drivetrain.balance()
        else:
            rightXCorrected = self.deadzoneCorrection(-driver.getRightX(), 0.55 * speedMulti)
            rightYCorrected = self.deadzoneCorrection(driver.getRightY(), 0.55 * speedMulti)
            leftXCorrected = self.deadzoneCorrection(driver.getLeftX(), 0.55 * speedMulti)
            # check if there's any input at all
            if rightXCorrected != 0 or rightYCorrected != 0 or leftXCorrected != 0:
                self.move(rightXCorrected, rightYCorrected, leftXCorrected)
                self.drivetrain.execute()
            elif self.drivetrain.getWheelLock():
                self.move(rightXCorrected, rightYCorrected, leftXCorrected)
                self.drivetrain.execute()
            else:
                self.drivetrain.idle()

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
    


    def teleopGrabber(self):
        operator = self.operator.xboxController
        #deadzone
        self.grabber.extend(self.deadzoneCorrection(operator.getLeftY(), operator.deadzone))
        if operator.getYButtonReleased():
            self.grabber.toggle()
    
    def teleopIntake(self):
        operator = self.operator.xboxController
        if operator.getXButtonReleased():
            self.intake.toggle()
        
    def autonomousInit(self):
        if not self.auton:
            return
        if not self.drivetrain:
            return
        if not self.swervometer:
            return

        self.autonTimer = wpilib.Timer()
        self.autonTimer.start()

        self.drivetrain.resetGyro()

    def autonomousPeriodic(self):
        if not self.auton:
            return
        if not self.drivetrain:
            return
        if not self.swervometer:
            return

        #print("autonomousPeriodic")
        x, y, rcw = self.swervometer.getCOF()
        print("auton: old position: x:", x, " y: ", y, " rcw: ", rcw)
        
        if (self.drivetrain.goToPose(15, 15, 0) == True):
            print("AUTON: Completed move to target.")
            x, y, rcw = self.swervometer.getCOF()
            print("auton: new position: x:", x, " y: ", y, " rcw: ", rcw)
        else:
            print("AUTON: Not yet at target.") 
            x, y, rcw = self.swervometer.getCOF()
            print("auton: new position: x:", x, " y: ", y, " rcw: ", rcw)
        print("============================================")
        
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