import math
import time
import sys
import os
from datetime import datetime

import wpilib
import wpilib.drive
import wpimath.controller
from wpilib import interfaces
import rev
import ctre
from navx import AHRS
from networktables import NetworkTables

from robotconfig import robotconfig, MODULE_NAMES
from controller import Controller
from swervedrive import SwerveDrive
from swervemodule import SwerveModule
from swervemodule import ModuleConfig

from swervedrive import BalanceConfig
from swervedrive import TargetConfig
from swervedrive import BearingConfig
from swervedrive import VisionDriveConfig

from swervometer import FieldConfig
from swervometer import RobotPropertyConfig
from swervometer import Swervometer

from cliffdetector import CliffDetector
from elevator import Elevator
from grabber import Grabber
from vision import Vision
from claw import Claw
from logger import Logger
from dashboard import Dashboard

from tester import Tester

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

# Test Mode
TEST_MODE = False

DASH_PREFIX = MODULE_NAMES.ROBOT

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):

        self.drivetrain = None
        self.swervometer = None
        self.driver = None
        self.operator = None
        self.tester = None
        self.cliffDetector = None
        self.auton = None
        self.vision = None
        self.elevator = None
        self.grabber = None
        self.claw = None

        # Even if no drivetrain, defaults to drive phase
        self.phase = "DRIVE_PHASE"

        self.config = robotconfig

        self.dashboard = Dashboard.getDashboard(testMode=TEST_MODE)

        dir = ''
        if TEST_MODE:
            dir = os.getcwd() # this doesn't work on mac, will write to python dir. Fix later.
        
        self.logger = self.initLogger(dir)
        self.log('Robot init; TEST_MODE =', TEST_MODE)

        for key, config in self.config.items():
            if key == 'CONTROLLERS':
                controllers = self.initControllers(config)
                self.driver = controllers[0]
                self.operator = controllers[1]
            if key == 'AUTON':
                self.auton = self.initAuton(config)
            if key == 'VISION':
                self.vision = self.initVision(config)
            if key == 'SWERVOMETER':
                self.swervometer = self.initSwervometer(config)
            if key == 'ELEVATOR':
                self.elevator = self.initElevator(config)
            if key == 'GRABBER':
                self.grabber = self.initGrabber(config)
            if key == 'DRIVETRAIN':
                self.drivetrain = self.initDrivetrain(config)
            if key == 'CLAW':
                self.claw = self.initClaw(config)

            #if key == 'CLIFFDETECTOR':
            #    self.cliffDetector = self.initCliffDetector(config)

        self.periods = 0

        if self.drivetrain:
            self.drivetrain.resetGyro()
            self.drivetrain.printGyro()

        self.elevator_has_reset = False

    def disabledExit(self):
        self.log("no longer disabled")
        if self.drivetrain:
            self.drivetrain.reset()

        if self.claw:
            self.claw.off()

        # Reset task counter.
        self.autonTaskCounter = 0
        self.maneuverTaskCounter = 0

        self.grabber.resetGrabber()

        self.elevator.resetElevator()

    def initLogger(self, dir):
        return Logger.getLogger(dir)
    
    def initControllers(self, config):
        ctrls = {}
        self.log(config)
        for ctrlConfig in config.values():
            self.log(ctrlConfig)
            controller_id = ctrlConfig['ID']
            ctrl = wpilib.XboxController(controller_id)
            dz = ctrlConfig['DEADZONE']
            lta = ctrlConfig['LEFT_TRIGGER_AXIS']
            rta = ctrlConfig['RIGHT_TRIGGER_AXIS']
            ctrls[controller_id] = Controller(ctrl, dz, lta, rta)
        return ctrls

    def initSwervometer(self, config):
        self.log("initSwervometer ran")
        
        if (config['TEAM_IS_RED']):
            self.team_is_red = True
            self.team_is_blu = False
            teamGyroAdjustment = 180 # Red Team faces 180 degrees at start.
            teamMoveAdjustment = 1 # Red Team start is oriented in the same direction as field.
        else:
            self.team_is_red = False
            self.team_is_blu = True
            teamGyroAdjustment = 0 # Blue Team faces 0 degrees at start.
            teamMoveAdjustment = -1 # Blue Team start is oriented 180 degrees from field.

        self.dashboard.putBoolean(DASH_PREFIX, 'Team is Red', self.team_is_red)

        self.log("FIELD_START_POSITION:", config['FIELD_START_POSITION'])

        if (config['FIELD_START_POSITION'] == 'A'):
            self.dashboard.putString(DASH_PREFIX, 'Field Start Position', 'A')
            self.fieldStartPosition = 'A'
            if self.team_is_red:
                starting_position_x = config['FIELD_RED_A_START_POSITION_X']
                starting_position_y = config['FIELD_RED_A_START_POSITION_Y']
                starting_angle = config['FIELD_RED_A_START_ANGLE']
            else: # self.team_is_blu
                starting_position_x = config['FIELD_BLU_A_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_A_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_A_START_ANGLE']
        elif (config['FIELD_START_POSITION'] == 'B'):
            self.dashboard.putString(DASH_PREFIX, 'Field Start Position', 'B')
            self.fieldStartPosition = 'B'
            if self.team_is_red:
                starting_position_x = config['FIELD_RED_B_START_POSITION_X']
                starting_position_y = config['FIELD_RED_B_START_POSITION_Y']
                starting_angle = config['FIELD_RED_B_START_ANGLE']
            else: # self.team_is_blu
                starting_position_x = config['FIELD_BLU_B_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_B_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_B_START_ANGLE']
        else: # config['FIELD_START_POSITION'] == 'C'
            self.dashboard.putString(DASH_PREFIX, 'Field Start Position', 'C')
            self.fieldStartPosition = 'C'
            if self.team_is_red:
                starting_position_x = config['FIELD_RED_C_START_POSITION_X']
                starting_position_y = config['FIELD_RED_C_START_POSITION_Y']
                starting_angle = config['FIELD_RED_C_START_ANGLE']
            else: # self.team_is_blu
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

        self.dashboard.putBoolean(DASH_PREFIX, 'Has Bumpers Attached', bumpers_attached)

        field_cfg = FieldConfig(sd_prefix='Field_Module',
                                origin_x=config['FIELD_ORIGIN_X'],
                                origin_y=config['FIELD_ORIGIN_Y'],
                                start_position_x= starting_position_x,
                                start_position_y= starting_position_y,
                                start_angle= starting_angle)
        
        robot_cfg = RobotPropertyConfig(sd_prefix='Robot_Property_Module',
                                is_red_team=self.team_is_red,
                                team_gyro_adjustment=teamGyroAdjustment,
                                team_move_adjustment=teamMoveAdjustment,
                                use_com_adjustment=config['USE_COM_ADJUSTMENT'],
                                frame_dimension_x=config['ROBOT_FRAME_DIMENSION_X'],
                                frame_dimension_y=config['ROBOT_FRAME_DIMENSION_Y'],
                                bumper_dimension_x=actual_bumper_dimension_x,
                                bumper_dimension_y=actual_bumper_dimension_y,
                                cof_offset_x=config['ROBOT_COF_OFFSET_X'],
                                cof_offset_y=config['ROBOT_COF_OFFSET_Y'],
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
    
    def initVision(self, config):
        vision = Vision(NetworkTables.getTable('limelight'),
                        config['APRILTAGS'],
                        config['RETROREFLECTIVE'],
                        config['MIN_TARGET_ASPECT_RATIO_REFLECTIVE'],
                        config['MAX_TARGET_ASPECT_RATIO_REFLECTIVE'],
                        config['MIN_TARGET_ASPECT_RATIO_APRILTAG'],
                        config['MAX_TARGET_ASPECT_RATIO_APRILTAG'],
                        config['UPDATE_POSE'])

        return vision

    def initElevator(self, config):
        elevator = Elevator(config['RIGHT_ID'], 
                            config['LEFT_ID'], 
                            config['SOLENOID_FORWARD_ID'], 
                            config['SOLENOID_REVERSE_ID'], 
                            config['ELEVATOR_KP'], 
                            config['ELEVATOR_KI'], 
                            config['ELEVATOR_KD'], 
                            config['LOWER_SAFETY'], 
                            config['UPPER_SAFETY'], 
                            self.grabber,
                            config['LEFT_LIMIT_SWITCH'],
                            config['RIGHT_LIMIT_SWITCH'])
        self.cone_elevator_human_position = config['CONE_ELEVATOR_HUMAN_POSITION']
        self.cone_elevator_upper_scoring_height = config['CONE_ELEVATOR_UPPER_SCORING_HEIGHT']
        self.cone_elevator_lower_scoring_height = config['CONE_ELEVATOR_LOWER_SCORING_HEIGHT']
        self.cone_elevator_retracted_height = config['CONE_ELEVATOR_RETRACTED_HEIGHT']
        self.cube_elevator_human_position = config['CUBE_ELEVATOR_HUMAN_POSITION']
        self.cube_elevator_upper_scoring_height = config['CUBE_ELEVATOR_UPPER_SCORING_HEIGHT']
        self.cube_elevator_lower_scoring_height = config['CUBE_ELEVATOR_LOWER_SCORING_HEIGHT']
        self.cube_elevator_retracted_height = config['CUBE_ELEVATOR_RETRACTED_HEIGHT']
        self.cube_elevator_destination = 0
        return elevator

    def initGrabber(self, config):
        self.cone_grabber_human_position = config['CONE_GRABBER_HUMAN_POSITION']
        self.cone_grabber_upper_scoring_height = config['CONE_GRABBER_UPPER_SCORING_HEIGHT']
        self.cone_grabber_lower_scoring_height = config['CONE_GRABBER_LOWER_SCORING_HEIGHT']
        self.cone_grabber_retracted_height = config['CONE_GRABBER_RETRACTED_HEIGHT']
        self.cube_grabber_human_position = config['CUBE_GRABBER_HUMAN_POSITION']
        self.cube_grabber_upper_scoring_height = config['CUBE_GRABBER_UPPER_SCORING_HEIGHT']
        self.cube_grabber_lower_scoring_height = config['CUBE_GRABBER_LOWER_SCORING_HEIGHT']
        self.cube_grabber_retracted_height = config['CUBE_GRABBER_RETRACTED_HEIGHT']
        return Grabber(config['ROTATE_MOTOR_ID'], config['GRABBER_ROTATE_SPEED'], config['GRABBER_KP'], config['GRABBER_KI'], config['GRABBER_KD'], config['MAX_POSITION'], config['MIN_POSITION'])

    def initClaw(self, config):
        return Claw(config['MOTOR_ID'], config['CONE_DEFAULT_RELEASE_SPEED'], config['CONE_UPPER_SCORING_HEIGHT_RELEASE_SPEED'], config['CONE_LOWER_SCORING_HEIGHT_RELEASE_SPEED'], config['CUBE_DEFAULT_RELEASE_SPEED'], config['CUBE_UPPER_SCORING_HEIGHT_RELEASE_SPEED'], config['CUBE_LOWER_SCORING_HEIGHT_RELEASE_SPEED'], config['RELEASE_CHANGE'], config['INTAKE_SPEED'], config['INTAKE_CHANGE'])
    
    def initDrivetrain(self, config):
        self.log("initDrivetrain ran")
        self.drive_type = config['DRIVETYPE']  # side effect!

        balance_cfg = BalanceConfig(sd_prefix='Balance_Module', balance_pitch_kP=config['BALANCE_PITCH_KP'], balance_pitch_kI=config['BALANCE_PITCH_KI'], balance_pitch_kD=config['BALANCE_PITCH_KD'], balance_yaw_kP=config['BALANCE_YAW_KP'], balance_yaw_kI=config['BALANCE_YAW_KI'], balance_yaw_kD=config['BALANCE_YAW_KD'])
        target_cfg = TargetConfig(sd_prefix='Target_Module', target_kP=config['TARGET_KP'], target_kI=config['TARGET_KI'], target_kD=config['TARGET_KD'])
        bearing_cfg = BearingConfig(sd_prefix='Bearing_Module', bearing_kP=config['BEARING_KP'], bearing_kI=config['BEARING_KI'], bearing_kD=config['BEARING_KD'])
        vision_cfg = VisionDriveConfig(sd_prefix='Vision_Module',
            x_visionDrive_kP=config['X_VISION_DRIVE_KP'], x_visionDrive_kI=config['X_VISION_DRIVE_KI'], x_visionDrive_kD=config['X_VISION_DRIVE_KD'],
            y_visionDrive_kP=config['Y_VISION_DRIVE_KP'], y_visionDrive_kI=config['Y_VISION_DRIVE_KI'], y_visionDrive_kD=config['Y_VISION_DRIVE_KD'],
            target_offsetX_reflective=config['REFLECTIVE_TARGET_OFFSET_X'], target_target_size_reflective=config['REFLECTIVE_TARGET_TARGET_SIZE'],
            target_offsetX_april=config['APRIL_TARGET_OFFSET_X'], target_target_size_april=config['APRIL_TARGET_TARGET_SIZE'],
            max_target_offset_x=config['MAX_TARGET_OFFSET_X'], min_target_size=config['MIN_TARGET_SIZE'])
    
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

        # Rotate motors
        flModule_rotateMotor = rev.CANSparkMax(config['FRONTLEFT_ROTATEMOTOR'], motor_type)
        frModule_rotateMotor = rev.CANSparkMax(config['FRONTRIGHT_ROTATEMOTOR'], motor_type)
        rlModule_rotateMotor = rev.CANSparkMax(config['REARLEFT_ROTATEMOTOR'], motor_type)
        rrModule_rotateMotor = rev.CANSparkMax(config['REARRIGHT_ROTATEMOTOR'], motor_type)

        flModule_rotateMotor_encoder = ctre.CANCoder(config['FRONTLEFT_ENCODER'])
        frModule_rotateMotor_encoder = ctre.CANCoder(config['FRONTRIGHT_ENCODER'])
        rlModule_rotateMotor_encoder = ctre.CANCoder(config['REARLEFT_ENCODER'])
        rrModule_rotateMotor_encoder = ctre.CANCoder(config['REARRIGHT_ENCODER'])

        frontLeftModule = SwerveModule(flModule_driveMotor, flModule_driveMotor_encoder, flModule_rotateMotor, flModule_rotateMotor_encoder, flModule_cfg)
        frontRightModule = SwerveModule(frModule_driveMotor, frModule_driveMotor_encoder, frModule_rotateMotor, frModule_rotateMotor_encoder, frModule_cfg)
        rearLeftModule = SwerveModule(rlModule_driveMotor, rlModule_driveMotor_encoder, rlModule_rotateMotor, rlModule_rotateMotor_encoder, rlModule_cfg)
        rearRightModule = SwerveModule(rrModule_driveMotor, rrModule_driveMotor_encoder, rrModule_rotateMotor, rrModule_rotateMotor_encoder, rrModule_cfg)

        # Set Open and Closed Loop Ramp Rate for Teleop
        self.teleopOpenLoopRampRate = config['TELEOP_OPEN_LOOP_RAMP_RATE']
        self.teleopClosedLoopRampRate = config['TELEOP_CLOSED_LOOP_RAMP_RATE']
        self.autonSteerStraight = config['AUTON_STEER_STRAIGHT']
        self.teleopSteerStraight = config['TELEOP_STEER_STRAIGHT']

        # Define maneuver task lists
        self.lowConeScoreTaskList = config['LOW_CONE_SCORE']
        self.highConeScoreTaskList = config['HIGH_CONE_SCORE']
        self.humanStationTaskList = config['HUMAN_STATION_PICKUP']
        self.rotateClockwiseTaskList = config['ROTATE_CLOCKWISE']
        self.rotateCounterclockwiseTaskList = config['ROTATE_COUNTERCLOCKWISE']

        #gyro = AHRS.create_spi()
        gyro = AHRS.create_spi(wpilib._wpilib.SPI.Port.kMXP, 500000, 50) # https://www.chiefdelphi.com/t/navx2-disconnecting-reconnecting-intermittently-not-browning-out/425487/36
        
        swerve = SwerveDrive(frontLeftModule, frontRightModule, rearLeftModule, rearRightModule, self.swervometer, self.vision, gyro, balance_cfg, target_cfg, bearing_cfg, vision_cfg, self.autonSteerStraight, self.teleopSteerStraight)

        return swerve

    def initAuton(self, config):
        self.autonScoreExisting = config['SCORE_EXISTING']
        self.autonBalanceRobot = config['BALANCE_BOT']
        self.autonDoCommunity = config['DO_COMMUNITY']

        self.dashboard.putNumber(DASH_PREFIX, 'Auton Score Existing Element', self.autonScoreExisting)
        self.dashboard.putNumber(DASH_PREFIX, 'Auton Balance Robot', self.autonBalanceRobot)

        # Reset task counter.
        self.autonTaskCounter = 0

        # Set Open Loop Ramp Rate for Auton
        self.autonOpenLoopRampRate = config['AUTON_OPEN_LOOP_RAMP_RATE']
        self.autonClosedLoopRampRate = config['AUTON_CLOSED_LOOP_RAMP_RATE']

        # Figure out task list
        if (self.team_is_red
            and self.fieldStartPosition == 'A'
            and self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_A_TF']
        elif (self.team_is_red
            and self.fieldStartPosition == 'A'
            and self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_A_TT']
        elif (self.team_is_red
            and self.fieldStartPosition == 'A'
            and not self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_A_FT']
        elif (self.team_is_red
            and self.fieldStartPosition == 'A'
            and not self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_A_FF']
        elif (self.team_is_red
            and self.fieldStartPosition == 'B'
            and self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_B_TF']     
        elif (self.team_is_red
            and self.fieldStartPosition == 'B'
            and self.autonScoreExisting
            and self.autonBalanceRobot):
                if self.autonDoCommunity:
                    self.autonTaskList = config['TASK_RED_B_TTT']
                else:
                    self.autonTaskList = config['TASK_RED_B_TTF']     
        elif (self.team_is_red
            and self.fieldStartPosition == 'B'
            and not self.autonScoreExisting
            and self.autonBalanceRobot):
                if self.autonDoCommunity:
                    self.autonTaskList = config['TASK_RED_B_FTT']
                else:
                    self.autonTaskList = config['TASK_RED_B_FTF']     
        elif (self.team_is_red
            and self.fieldStartPosition == 'B'
            and not self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_B_FF']     
        elif (self.team_is_red
            and self.fieldStartPosition == 'C'
            and self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_C_TF']
        elif (self.team_is_red
            and self.fieldStartPosition == 'C'
            and self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_C_TT']
        elif (self.team_is_red
            and self.fieldStartPosition == 'C'
            and not self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_C_FT']
        elif (self.team_is_red
            and self.fieldStartPosition == 'C'
            and not self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_C_FF']
        elif (not self.team_is_red
            and self.fieldStartPosition == 'A'
            and self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_A_TF']     
        elif (not self.team_is_red
            and self.fieldStartPosition == 'A'
            and self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_A_TT']     
        elif (not self.team_is_red
            and self.fieldStartPosition == 'A'
            and not self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_A_FT']     
        elif (not self.team_is_red
            and self.fieldStartPosition == 'A'
            and not self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_A_FF']     
        elif (not self.team_is_red
            and self.fieldStartPosition == 'B'
            and self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_B_TF']
        elif (not self.team_is_red
            and self.fieldStartPosition == 'B'
            and self.autonScoreExisting
            and self.autonBalanceRobot):
                if self.autonDoCommunity:
                    self.autonTaskList = config['TASK_BLU_B_TTT']
                else:
                    self.autonTaskList = config['TASK_BLU_B_TTF']     
        elif (not self.team_is_red
            and self.fieldStartPosition == 'B'
            and not self.autonScoreExisting
            and self.autonBalanceRobot):
                if self.autonDoCommunity:
                    self.autonTaskList = config['TASK_BLU_B_FTT']
                else:
                    self.autonTaskList = config['TASK_BLU_B_FTF']     
        elif (not self.team_is_red
            and self.fieldStartPosition == 'B'
            and not self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_B_FF']
        elif (not self.team_is_red
            and self.fieldStartPosition == 'C'
            and self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_C_TF']
        elif (not self.team_is_red
            and self.fieldStartPosition == 'C'
            and self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_C_TT']
        elif (not self.team_is_red
            and self.fieldStartPosition == 'C'
            and self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_C_TF']
        elif (not self.team_is_red
            and self.fieldStartPosition == 'C'
            and not self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_C_FF']
        else: # No matching task list
            self.autonTaskCounter = -1
            self.autonTaskList = []

        self.log("Init Auton: Task List: ", self.autonTaskList)
        return True

    def initCliffDetector(self, config):
        self.log(config)
        cliffDetector = CliffDetector(
            config['LEFT_CLIFF_DETECTOR_PINGID'], 
            config['LEFT_CLIFF_DETECTOR_ECHOID'], 
            config['RIGHT_CLIFF_DETECTOR_PINGID'],
            config['RIGHT_CLIFF_DETECTOR_ECHOID'],
            config['CLIFF_TOLERANCE'])
        return cliffDetector

    def robotPeriodic(self):
        #if self.cliffDetector:
        #    self.cliffDetector.update()
        return True

    def teleopInit(self):
        self.log("teleopInit ran")

        self.drivetrain.setRampRates(self.teleopOpenLoopRampRate, self.teleopClosedLoopRampRate)
        self.maneuverComplete = True
        self.startingManeuver = True
        self.maneuverTaskCounter = 0

        self.drivetrain.setInAuton(False)
        
        return True

    def teleopPeriodic(self):

        operator = self.operator.xboxController

        # Use only if limit switches BOTH break.
        if (operator.getLeftBumper() and operator.getRightBumper()):
            print("Elevator: Bypassing Elevator Lower Limit Switches")
            self.elevator.bypassLimitSwitch()
            print("Grabber: Bypassing Grabber Upper Limit Switches")
            self.grabber.bypassLimitSwitch()
            return

        #elf.log("TeleopPeriodic: hasGrabberReset: ", self.grabber.hasGrabberReset())
        if self.grabber.hasGrabberReset() == False:
            self.grabber.grabberReset()
            return
        #self.log("TeleopPeriodic: Grabber reset test complete")
        
        #self.log("TeleopPeriodic: hasElevatorReset: ", self.elevator.hasElevatorReset())
        if self.elevator.hasElevatorReset() == False:
            self.elevator.elevatorReset()
            self.grabber.update()
            return
        #self.log("TeleopPeriodic: Elevator reset test complete")
        
        if self.teleopDrivetrain():
            self.log("TeleoDrivetrain returned true. In a maneuver.")
            return
        else:
            self.log("TeleoDrivetrain returned False. Not in a maneuver.")
            self.teleopElevatorGrabber()
            self.teleopClaw()
            return

    def teleopDrivetrain(self):
        if (not self.drivetrain):
            return False
        if (not self.driver):
            return False

        driver = self.driver.xboxController
        deadzone = self.driver.deadzone

        # Implement clutch on driving and rotating.
        translational_clutch = 1.0
        rotational_clutch = 1.0
        if (driver.getRightBumper()):
            translational_clutch = 0.4
            rotational_clutch = 0.4
        if (driver.getLeftBumper()): # This is deliberately an "if", not an "elif", to aid in driver transition.
            translational_clutch = 0.2
            rotational_clutch = 0.3 #0.2 was a little too slow for rotation, but perfect for translation

        # Reset the gyro in the direction bot is facing.
        # Note this is a bad idea in competition, since it's reset automatically in robotInit.
        if (driver.getLeftTriggerAxis() > 0.7 and driver.getRightTriggerAxis() > 0.7 and driver.getXButton()):
            self.drivetrain.resetGyro()
            self.drivetrain.printGyro()

        # Determine if Wheel Lock is needed.
        if (driver.getLeftTriggerAxis() > 0.7 and not driver.getRightTriggerAxis() > 0.7):
            self.drivetrain.setWheelLock(True)
        else:
            self.drivetrain.setWheelLock(False)
        
        #Manuevers
        rcw = self.deadzoneCorrection(driver.getRightX() * rotational_clutch, self.driver.deadzone)
        if(driver.getAButton()):
            self.drivetrain.balance()
            return False
        elif(driver.getYButton()):
            self.drivetrain.goToReflectiveTapeCentered()
            return False
        #elif (driver.getBButton()):
        #    if(self.startingManeuver == True):
        #        self.log("B Button - Starting Maneuver")
        #        self.startingManeuver = False
        #        self.maneuverComplete = False
        #        self.maneuverTaskCounter = 0
        #        self.maneuverTaskList = self.lowConeScoreTaskList
        #    self.teleopManeuver()
        #    return True
        #elif (driver.getYButton()):
        #    if(self.startingManeuver == True):
        #        self.log("Y Button - Starting Maneuver")
        #        self.startingManeuver = False
        #        self.maneuverComplete = False
        #        self.maneuverTaskCounter = 0
        #        self.maneuverTaskList = self.highConeScoreTaskList
        #    self.teleopManeuver()
        #    return True
        #elif (driver.getXButton()):
        #    if(self.startingManeuver == True):
        #        self.log("X Button - Starting Maneuver")
        #        self.startingManeuver = False
        #        self.maneuverComplete = False
        #        self.maneuverTaskCounter = 0
        #        self.maneuverTaskList = self.humanStationTaskList
        #    self.teleopManeuver()
        #    return True
        elif (driver.getRightTriggerAxis() > 0.7 and rcw > 0):
            if(self.startingManeuver == True):
                print("180 Clockwise Flip - Starting Maneuver")
                self.startingManeuver = False
                self.maneuverComplete = False
                self.maneuverTaskCounter = 0
                self.maneuverTaskList = self.rotateClockwiseTaskList
            self.teleopManeuver()
            return True
        elif (driver.getRightTriggerAxis() > 0.7 and rcw < 0):
            if(self.startingManeuver == True):
                print("180 Counterclockwise Flip - Starting Maneuver")
                self.startingManeuver = False
                self.maneuverComplete = False
                self.maneuverTaskCounter = 0
                self.maneuverTaskList = self.rotateCounterclockwiseTaskList
            self.teleopManeuver()
            return True
        elif (driver.getAButton() == False and driver.getBButton() == False and driver.getYButton() == False and driver.getXButton() and self.maneuverComplete == True):
            self.startingManeuver = True
            return True
        
        # Regular driving, not a maneuver
        else:
            strafe = self.deadzoneCorrection(driver.getLeftX() * translational_clutch, self.driver.deadzone)
            fwd = self.deadzoneCorrection(driver.getLeftY() * translational_clutch, self.driver.deadzone)
            rcw = self.deadzoneCorrection(driver.getRightX() * rotational_clutch, self.driver.deadzone)
            
            fwd *= -1 # Because controller is backwards from you think
            
            # Bot starts facing controller
            controller_at_180_to_bot = -1
            fwd *= controller_at_180_to_bot
            strafe *= controller_at_180_to_bot

            # Need to adjust for Team:
            fwd *= self.swervometer.getTeamMoveAdjustment()
            strafe *= self.swervometer.getTeamMoveAdjustment()

            # No need to correct RCW, as clockwise is clockwise whether you are facing with or against bot.
            
            # If any joysticks are dictating movement.
            if fwd != 0 or strafe != 0 or rcw != 0:
                self.drivetrain.move(fwd, strafe, rcw, self.drivetrain.getBearing())
                
                self.log("TeleopDriveTrain: POV: ", driver.getPOV())
                if self.getPOVCorner(driver.getPOV()) == 'front_left':
                    self.drivetrain.execute('front_left')
                elif self.getPOVCorner(driver.getPOV()) == 'front_right':
                    self.drivetrain.execute('front_right')
                elif self.getPOVCorner(driver.getPOV()) == 'rear_left':
                    self.drivetrain.execute('rear_left')
                elif self.getPOVCorner(driver.getPOV()) == 'rear_right':
                    self.drivetrain.execute('rear_right')
                else:
                    self.drivetrain.execute('center')
            # If no joysticks are dictating movement, but we want to lock the wheels.
            elif self.drivetrain.getWheelLock():
                self.drivetrain.move(0, 0, 0, self.drivetrain.getBearing())
                self.drivetrain.execute('center')
            # Otherwise, make sure we are explicitly doing nothing, so bot does not drift.
            else:
                self.drivetrain.idle()

        return False

    def getPOVCorner(self, value):
        if (value >=0 and value < 90):
            return 'front_right'
        elif (value >=90 and value < 180):
            return 'rear_right'
        elif (value >=180 and value < 270):
            return 'rear_left'
        elif (value >= 270 and value < 360):
            return 'front_left'
        else:
            return 'center'

    def teleopElevatorGrabber(self):
        if (not self.elevator):
            return
        if (not self.grabber):
            return
        
        operator = self.operator.xboxController

        self.log("teleopElevatorGrabber: In teleopElevatorGrabber()")

        if (operator.getLeftBumper() and not operator.getRightBumper()):
            self.log("teleopElevator: Toggling Elevator Up/Down")
            self.elevator.toggle()

        operator_clutch = 1
        #Check for clutch
        #if(operator.getLeftTriggerAxis() > 0.7):
        #    operator_clutch = 0.4
        
        #Find the value the arm will move at
        elevator_controller_value = (self.deadzoneCorrection(operator.getLeftY(), self.operator.deadzone) / 5) * operator_clutch
        grabber_controller_value = (self.deadzoneCorrection(operator.getRightY(), self.operator.deadzone)) * self.grabber.getRotateSpeed()

        if elevator_controller_value != 0 and grabber_controller_value != 0: # Move both in direction of controller
            self.grabber.move_grabber(grabber_controller_value)
            self.elevator.move(elevator_controller_value)
            self.log("ElevatorGrabber: Move both elevator and grabber.")
        elif elevator_controller_value !=0: # Move only elevator
            if not (operator.getLeftTriggerAxis() > 0.7):
                self.grabber.update() # Grabber stay in place
            self.elevator.move(elevator_controller_value)
            self.log("ElevatorGrabber: Move elevator, maintain grabber.")
        elif grabber_controller_value != 0: # Move only grabber
            self.grabber.move_grabber(grabber_controller_value)
            if not (operator.getLeftTriggerAxis() > 0.7):
                self.elevator.moveToPos(self.elevator.getTargetPosition()) # Elevator stay in place
            self.log("ElevatorGrabber: Move grabber, maintain elevator.")
        elif operator.getAButton() and operator.getLeftTriggerAxis() > 0.7: #Lowest Position - Cube
            self.claw.setCubeSpeedDefault()
            self.grabber.goToPosition(self.cube_grabber_retracted_height)
            self.elevator.moveToPos(self.cube_elevator_retracted_height)
            self.log("ElevatorGrabber: Cube: A Button")
        elif operator.getAButton(): #Lowest Position - Cone
            self.claw.setConeSpeedDefault()
            self.grabber.goToPosition(self.cone_grabber_retracted_height)
            self.elevator.moveToPos(self.cone_elevator_retracted_height)
            self.log("ElevatorGrabber: Cone: A Button")
        elif operator.getYButton() and operator.getLeftTriggerAxis() > 0.7: # and self.elevator.isElevatorDown(): #Highest Position
            self.claw.setCubeSpeedUpper()
            self.grabber.goToPosition(self.cube_grabber_upper_scoring_height)
            self.elevator.moveToPos(self.cube_elevator_upper_scoring_height)
            self.log("ElevatorGrabber: Cube: Y Button")
        elif operator.getYButton(): # and self.elevator.isElevatorDown(): #Highest Position
            self.claw.setConeSpeedUpper()
            self.grabber.goToPosition(self.cone_grabber_upper_scoring_height)
            self.elevator.moveToPos(self.cone_elevator_upper_scoring_height)
            self.log("ElevatorGrabber: Cone: Y Button")
        elif operator.getBButton() and operator.getLeftTriggerAxis() > 0.7: # and self.elevator.isElevatorDown(): #Medium Position
            self.claw.setCubeSpeedLower()
            self.grabber.goToPosition(self.cube_grabber_lower_scoring_height)
            self.elevator.moveToPos(self.cube_elevator_lower_scoring_height)
            self.log("ElevatorGrabber: Cube: B Button")
        elif operator.getBButton(): # and self.elevator.isElevatorDown(): #Medium Position
            self.claw.setConeSpeedLower()
            self.grabber.goToPosition(self.cone_grabber_lower_scoring_height)
            self.elevator.moveToPos(self.cone_elevator_lower_scoring_height)
            self.log("ElevatorGrabber: Cone: B Button")
        elif operator.getXButton() and operator.getLeftTriggerAxis() > 0.7: # and self.elevator.isElevatorDown(): #Human Position
            self.claw.setCubeSpeedDefault()
            self.grabber.goToPosition(self.cube_grabber_human_position)
            self.elevator.moveToPos(self.cube_elevator_human_position)
            self.log("ElevatorGrabber: Cube: X Button")
        elif operator.getXButton(): # and self.elevator.isElevatorDown(): #Human Position
            self.claw.setConeSpeedDefault()
            self.grabber.goToPosition(self.cone_grabber_human_position)
            self.elevator.moveToPos(self.cone_elevator_human_position)
            self.log("ElevatorGrabber: Cone: X Button")
        else: #Aim for last target.
            #if (operator.getLeftTriggerAxis() > 0.7):
            #    self.grabber.motor_off()
            #    self.elevator.motors_off()
            #else:
            self.grabber.update() # Grabber stay in place with PID
                #self.grabber.goToPosition(self.grabber.getTargetRotatePosition())
            self.elevator.moveToPos(self.elevator.getTargetPosition()) #Elevator stay in place with PID
            self.log("ElevatorGrabber: Exiting after maintaining position.")
        
        return

    def teleopClaw(self):
        operator = self.operator.xboxController
        if (operator.getRightBumper()):
            self.log("Claw: Intake")
            self.claw.intake()
        elif (operator.getRightTriggerAxis() > 0.7):
            self.log("Claw: Release")
            self.log("Claw: At time of release: Elevator Position: ", self.elevator.getEncoderPosition(), " Grabber Position: ", self.grabber.getEncoderPosition())
            self.claw.release()
        else:
            self.claw.off()
        
    def autonomousInit(self):
        if not self.auton:
            return
        if not self.drivetrain:
            return
        if not self.swervometer:
            return

        self.autonTimer = wpilib.Timer()
        self.autonTimer.start()

        self.drivetrain.setInAuton(True)

        self.drivetrain.resetGyro()
        if self.team_is_red:
            self.drivetrain.setBearing(180)
        else:
            self.drivetrain.setBearing(0)
        self.drivetrain.setRampRates(self.autonOpenLoopRampRate, self.autonClosedLoopRampRate)

        # Reset the task counter
        self.autonTaskCounter = 0

    def autonomousPeriodic(self):
        if not self.auton:
            return
        if not self.drivetrain:
            return
        if not self.swervometer:
            return
        if not self.autonTimer:
            return

        self.log("AutonomousPeriodic: hasGrabberReset: ", self.grabber.hasGrabberReset())
        if self.grabber.hasGrabberReset() == False:
            self.grabber.grabberReset()
            return
        self.log("AutonomousPeriodic: Grabber reset test complete")
        
        self.log("AutonomousPeriodic: hasElevatorReset: ", self.elevator.hasElevatorReset())
        if self.elevator.hasElevatorReset() == False:
            self.elevator.elevatorReset()
            self.grabber.update()
            return
        self.log("AutonomousPeriodic: Elevator reset test complete")
        
        if self.autonTaskCounter < 0:
            return # No tasks assigned.

        if self.autonTaskCounter >= len(self.autonTaskList):
            return # No tasks remaining.
            
        autonTask = self.autonTaskList[self.autonTaskCounter]

        self.log("Autonomous Periodic: Past Grabber reset and Elevator reset ")
        self.log("Current Task Counter: ", self.autonTaskCounter, "Current Task: ", autonTask, "Current Task List: ", self.autonTaskList)

        if (autonTask[0] == 'TIMER'):
            self.log("RUNNING Auton Task TIMER, timer: ", self.autonTimer.get())
            if self.autonTimer.get() > autonTask[1]:
                self.log("ENDING Auton Task: TIMER: ", self.autonTimer.get())
                self.autonTaskCounter += 1
            self.grabber.update()
            self.elevator.update()
        elif (autonTask[0] == 'CLAW_INTAKE_AND_STOP'):
            self.log("RUNNING Auton Task CLAW_INTAKE_AND_STOP, autonTaskCounter: ", self.autonTaskCounter)
            if self.claw.runAndStop(-1):
                self.log("ENDING Auton Task CLAW_INTAKE_AND_STOP")
                self.autonTaskCounter += 1
            self.grabber.update()
            self.elevator.update()
        elif (autonTask[0] == 'CLAW_RELEASE_AND_STOP'):
            self.log("RUNNING Auton Task CLAW_RELEASE_AND_STOP, autonTaskCounter: ", self.autonTaskCounter)
            if self.claw.runAndStop(+1):
                self.log("ENDING Auton Task CLAW_RELEASE_AND_STOP")
                self.autonTaskCounter += 1
            self.grabber.update()
            self.elevator.update()
        elif (autonTask[0] == 'CLAW_STOP'):
            self.log("RUNNING Auton Task CLAW_STOP, autonTaskCounter: ", self.autonTaskCounter)
            if self.claw.off():
                self.log("ENDING Auton Task CLAW_STOP")
                self.autonTaskCounter += 1
            self.grabber.update()
            self.elevator.update()
        elif (autonTask[0] == 'RAISE_GRABBER'):
            self.log("RUNNING Auton Task RAISE_GRABBER, autonTaskCounter: ", self.autonTaskCounter)
            if self.grabber.raise_motor(self.grabber.getRotateSpeed()):
                self.log("ENDING Auton Task RAISE_GRABBER")
                self.autonTaskCounter += 1
            self.elevator.update()
        elif (autonTask[0] == 'LOWER_GRABBER'):
            self.log("RUNNING Auton Task LOWER_GRABBER, autonTaskCounter: ", self.autonTaskCounter)
            if self.grabber.lower_motor(self.grabber.getRotateSpeed()):
                self.log("ENDING Auton Task LOWER_GRABBER")
                self.autonTaskCounter += 1 
            self.elevator.update()
        elif (autonTask[0] == 'ELEVATOR_TOGGLE'):
            self.log("RUNNING Auton Task ELEVATOR_TOGGLE, autonTaskCounter: ", self.autonTaskCounter)
            if self.elevator.toggle():
                self.autonTaskCounter += 1
                self.log("ENDING Auton Task ELEVATOR_TOGGLE")
            self.grabber.update()
            self.elevator.update()
        elif (autonTask[0] == 'ELEVATOR_UP'):
            self.log("RUNNING Auton Task ELEVATOR_UP, autonTaskCounter: ", self.autonTaskCounter)
            self.log("Auton: Elevator Up: ", self.elevator.getEncoderPosition())
            if self.elevator.elevatorUp():
                self.log("ENDING Auton Task ELEVATOR_UP")
                self.autonTaskCounter += 1
            self.grabber.update()
            self.elevator.update()
        elif (autonTask[0] == 'ELEVATOR_DOWN'):
            self.log("RUNNING Auton Task ELEVATOR_DOWN, autonTaskCounter: ", self.autonTaskCounter)
            self.log("Auton: Elevator Down: ", self.elevator.getEncoderPosition())
            if self.elevator.elevatorDown():
                self.log("ENDING Auton Task ELEVATOR_DOWN")
                self.autonTaskCounter += 1
            self.grabber.update()
            self.elevator.update()
        elif (autonTask[0] == 'ELEVATOR_UPPER_EXTEND'):
            self.log("RUNNING Auton Task ELEVATOR_UPPER_EXTEND, autonTaskCounter: ", self.autonTaskCounter)
            if self.elevator.moveToPos(self.cube_elevator_upper_scoring_height): # and self.grabber.goToPosition(self.cube_grabber_upper_scoring_height):
                self.log("ENDING Auton Task ELEVATOR_UPPER_EXTEND")
                self.autonTaskCounter += 1
            self.grabber.goToPosition(self.cube_grabber_upper_scoring_height)
            self.claw.setCubeSpeedUpper()
            self.log("Auton: Elevator Upper Extend: ", self.elevator.getEncoderPosition())
        elif (autonTask[0] == 'ELEVATOR_LOWER_EXTEND'):
            self.log("RUNNING Auton Task ELEVATOR_LOWER_EXTEND, autonTaskCounter: ", self.autonTaskCounter)
            if self.elevator.moveToPos(self.cube_elevator_lower_scoring_height): #and self.grabber.goToPosition(self.cube_grabber_lower_scoring_height):
                self.log("ENDING Auton Task ELEVATOR_LOWER_EXTEND")
                self.autonTaskCounter += 1
            self.grabber.goToPosition(self.cube_grabber_lower_scoring_height)
            self.claw.setCubeSpeedLower()
            self.log("Auton: Elevator Lower Extend: ", self.elevator.getEncoderPosition())
        elif (autonTask[0] == 'ELEVATOR_RETRACT'):
            self.log("RUNNING Auton Task ELEVATOR_RETRACT, autonTaskCounter: ", self.autonTaskCounter)
            if self.elevator.moveToPos(self.cube_elevator_retracted_height): #and self.grabber.goToPosition(self.cube_grabber_retracted_height):
                self.log("ENDING Auton Task ELEVATOR_RETRACT")
                self.autonTaskCounter += 1
            self.grabber.goToPosition(self.cube_grabber_retracted_height)
            self.claw.setCubeSpeedDefault()
            self.log("Auton: Elevator Retract: ", self.elevator.getEncoderPosition())
        elif (autonTask[0] == 'MOVE'):
            x = autonTask[1]
            y = autonTask[2]
            bearing = autonTask[3]
            self.log("RUNNING Auton Task MOVE, autonTaskCounter: ", self.autonTaskCounter)
            self.log("Auton: Move: ", self.autonTaskCounter, " Target: x: ", x, " y: ", y, " bearing: ", bearing)
            if self.drivetrain.goToPose(x, y, bearing):
                self.log("ENDING Auton Task MOVE")
                self.log("Auton: Move: Reached target: x: ", x, " y: ", y, " bearing: ", bearing)
                self.autonTaskCounter += 1 # Move on to next task.
            else:
                # Leave self.autonTaskCounter unchanged. Repeat this task.
                self.log("Auton: Move: Not at target: x: ", x, " y: ", y, " bearing: ", bearing)    
            self.grabber.goToPosition(self.cube_grabber_retracted_height)
            self.elevator.moveToPos(self.cube_elevator_retracted_height)
        elif (autonTask[0] == 'MOVE_TO_BALANCE'):
            self.log("RUNNING Auton Task MOVE_TO_BALANCE, autonTaskCounter: ", self.autonTaskCounter)            
            x = autonTask[1]
            y = autonTask[2]
            bearing = autonTask[3]
            tolerance = autonTask[4]
            self.log("Auton: MOVE_TO_BALANCE: Target: x: ", x, " y: ", y, " bearing: ", bearing)
            if self.drivetrain.goToBalance(x, y, bearing, tolerance):
                self.log("Auton: MOVE_TO_BALANCE: Reached target: x: ", x, " y: ", y, " bearing: ", bearing)
                self.autonTaskCounter += 1 # Move on to next task.
            else:
                # Leave self.autonTaskCounter unchanged. Repeat this task.
                self.log("Auton: MOVE_TO_BALANCE: Not at target: x: ", x, " y: ", y, " bearing: ", bearing)
            self.grabber.goToPosition(self.cube_grabber_retracted_height)
            self.elevator.moveToPos(self.cube_elevator_retracted_height)
        elif (autonTask[0] == 'HALF_MOON_BALANCE'):
            self.log("RUNNING Auton Task HALF_MOON_BALANCE, autonTaskCounter: ", self.autonTaskCounter)            
            checkpointX = autonTask[1]
            checkpointY = autonTask[2]
            cornerX = autonTask[3]
            cornerY = autonTask[4]
            bearing = autonTask[5]
            tolerance = autonTask[6]
            self.log("Auton: HALF_MOON_BALANCE: Checkpoint x: ", checkpointX, " y: ", checkpointY, " Corner x: ", cornerX, " y: ", cornerY, " bearing: ", bearing, " tolerance: ", tolerance)
            if self.drivetrain.halfMoonBalance(checkpointX, checkpointY, cornerX, cornerY, bearing, tolerance):
                self.log("Auton: HALF_MOON_BALANCE: Reached target: Checkpoint x: ", checkpointX, " y: ", checkpointY, " Corner x: ", cornerX, " y: ", cornerY, " bearing: ", bearing, " tolerance: ", tolerance)
                self.autonTaskCounter += 1 # Move on to next task.
            else:
                # Leave self.autonTaskCounter unchanged. Repeat this task.
                self.log("Auton: HALF_MOON_BALANCE: Not at target: Checkpoint x: ", checkpointX, " y: ", checkpointY, " Corner x: ", cornerX, " y: ", cornerY, " bearing: ", bearing, " tolerance: ", tolerance)
            
            self.grabber.goToPosition(self.cube_grabber_retracted_height)
            self.elevator.moveToPos(self.cube_elevator_retracted_height)
        elif (autonTask[0] == 'BALANCE'):
            self.log("RUNNING Auton Task BALANCE, autonTaskCounter: ", self.autonTaskCounter)            

            if self.drivetrain.balance():
                self.log("ENDING Auton Task BALANCE")
                self.autonTaskCounter += 1 # Move on to next task.
                self.log("Auton: Balance: Leveled and oriented")
            else:
                # Leave self.autonTaskCounter unchanged. Repeat this task.
                self.log("Auton: Balance: Keep balancing and orienting")
            self.grabber.goToPosition(self.cube_grabber_retracted_height)
            self.elevator.moveToPos(self.cube_elevator_retracted_height)
        elif (autonTask[0] == 'WHEEL_LOCK'):
            self.log("RUNNING Auton Task WHEEL_LOCK, autonTaskCounter: ", self.autonTaskCounter)            
            self.drivetrain.setWheelLock(True)
            self.drivetrain.goToPose(0, 0, self.drivetrain.getBearing())
            self.log("ENDING Auton Task WHEEL_LOCK")
            self.autonTaskCounter += 1 # Move on to next task.
            self.grabber.goToPosition(self.cube_grabber_retracted_height)
            self.elevator.moveToPos(self.cube_elevator_retracted_height)
        elif (autonTask[0] == 'IDLE'):
            ## this will never end?
            self.log("RUNNING Auton Task IDLE, autonTaskCounter: ", self.autonTaskCounter)            
            self.drivetrain.idle()
            self.grabber.goToPosition(self.cube_grabber_retracted_height)
            self.elevator.moveToPos(self.cube_elevator_retracted_height)
        else:
            self.log("Auton: ERROR: Unknown Task", self.autonTaskCounter, autonTask[0])
            self.autonTaskCounter += 1   
            self.drivetrain.idle()
            self.grabber.goToPosition(self.cube_grabber_retracted_height)
            self.elevator.moveToPos(self.cube_elevator_retracted_height)
        
        return

    def teleopManeuver(self):
        if not self.drivetrain:
            return
        if not self.swervometer:
            return
        
        self.log("teleopManeuver")

        if self.maneuverTaskCounter < 0:
            return # No tasks assigned.

        if self.maneuverTaskCounter >= len(self.maneuverTaskList):
            self.maneuverComplete = True
            self.startingManeuver = True
            return # No tasks remaining.
            
        maneuverTask = self.maneuverTaskList[self.maneuverTaskCounter]

        self.log("WHICH TASK: ", maneuverTask[0])

        if (maneuverTask[0] == 'CLAW_OFF'):
            self.claw.off()
            self.maneuverTaskCounter += 1
            self.log("RUNNING AND ENDING Maneuver: Claw Off: ")
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'CLAW_INTAKE'):
            self.claw.intake()
            self.maneuverTaskCounter += 1
            self.log("RUNNING AND ENDING Maneuver: Claw Intake")
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'CLAW_RELEASE'):
            self.claw.release()
            self.maneuverTaskCounter += 1
            self.log("RUNNING AND ENDING Maneuver: Claw Release")
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'RAISE_GRABBER'):
            if self.grabber.raise_motor(self.grabber.getRotateSpeed()):
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.maneuverTaskCounter += 1
            self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.elevator.update()
        elif (maneuverTask[0] == 'LOWER_GRABBER'):
            if self.grabber.lower_motor(self.grabber.getRotateSpeed()):
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.maneuverTaskCounter += 1
            self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.elevator.update()
        elif (maneuverTask[0] == 'POSITION_GRABBER'):
            if self.grabber.goToPosition(maneuverTask[1]):
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.maneuverTaskCounter += 1
            self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.log("Position Grabber target position: ", maneuverTask[1])
            self.elevator.update()
        elif (maneuverTask[0] == 'ELEVATOR_TOGGLE'):
            if self.elevator.toggle():
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.maneuverTaskCounter += 1
            self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.log("Elevator Toggle: ", self.elevator.getEncoderPosition())
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'ELEVATOR_UP'):
            if self.elevator.elevatorUp():
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.maneuverTaskCounter += 1
            self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.log("Elevator position:", self.elevator.getEncoderPosition())
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'ELEVATOR_DOWN'):
            if self.elevator.elevatorDown():
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.maneuverTaskCounter += 1
            self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.log("Elevator position", self.elevator.getEncoderPosition())
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'ELEVATOR_LOWER_EXTEND'):
            if self.elevator.moveToPos(self.cone_elevator_lower_scoring_height) and self.grabber.goToPosition(self.cone_grabber_lower_scoring_height):
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.maneuverTaskCounter += 1
            self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.log("lower scoring height ", self.lower_scoring_height, " current position: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'ELEVATOR_HUMAN_EXTEND'):
            if self.elevator.moveToPos(self.cone_elevator_human_position)and self.grabber.goToPosition(self.cone_grabber_human_position):
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.maneuverTaskCounter += 1
            self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.log("elevator position: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'ELEVATOR_UPPER_EXTEND'):
            if self.elevator.moveToPos(self.cone_elevator_upper_scoring_height) and self.grabber.goToPosition(self.cone_grabber_upper_scoring_height):
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.maneuverTaskCounter += 1
            self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.log("elevator position: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'ELEVATOR_RETRACT'):
            if self.elevator.moveToPos(self.cone_elevator_retracted_height) and self.grabber.goToPosition(self.cone_grabber_retracted_height):
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.maneuverTaskCounter += 1
            self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.log("elevator position: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'MOVE'):
            x = maneuverTask[1]
            y = maneuverTask[2]
            bearing = maneuverTask[3]
            
            self.log("Moving to Target: x: ", x, " y: ", y, " bearing: ", bearing)
            if self.drivetrain.goToPose(x, y, bearing):
                self.maneuverTaskCounter += 1 # Move on to next task.
                self.log("ENDING Maneuver", maneuverTask[0])                            
                self.log("MOVE: Reached target: x: ", x, " y: ", y, " bearing: ", bearing)
            else:
                # Leave self.maneuverTaskCounter unchanged. Repeat this task.
                self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
                self.log("MOVE: Not at target: x: ", x, " y: ", y, " bearing: ", bearing)
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'MOVE_BACK'):
            backDistance = maneuverTask[1]
            x, y, bearing = self.swervometer.getCOF()
            if(self.team_is_red):
                x -= backDistance
            else:
                x += backDistance
            if self.drivetrain.goToPose(x, y, bearing):
                self.maneuverTaskCounter += 1 # Move on to next task.
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.log("MOVE_BACK: Reached target: x: ", x, " y: ", y, " bearing: ", bearing)
            else:
                # Leave self.maneuverTaskCounter unchanged. Repeat this task.
                self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
                self.log("MOVE_BACK: Not at target: x: ", x, " y: ", y, " bearing: ", bearing)
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'ROTATE'):
            degrees = maneuverTask[1]
            x, y, bearing = self.swervometer.getCOF()
            bearing = (bearing + degrees) % 360
            if self.drivetrain.goToPose(x, y, bearing):
                self.maneuverTaskCounter += 1 # Move on to next task.
                self.log("ENDING Maneuver", maneuverTask[0])            
                self.log("ROTATE: Reached target: x: ", x, " y: ", y, " bearing: ", bearing)
            else:
                # Leave self.maneuverTaskCounter unchanged. Repeat this task.
                self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
                self.log("ROTATE: Not at target: x: ", x, " y: ", y, " bearing: ", bearing)
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'BALANCE'):
            self.log("RUNNING Maneuver: BALANCE: ", self.maneuverTaskCounter)
            if self.drivetrain.balance():
                self.maneuverTaskCounter += 1 # Move on to next task.
                self.log("ENDING Maneuver", maneuverTask[0])            
            else:
                # Leave self.maneuverTaskCounter unchanged. Repeat this task.
                self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'ALIGN_TO_POLE'):
            if self.drivetrain.goToReflectiveTapeCentered():
                self.maneuverTaskCounter += 1 # Move on to next task.
                self.log("ENDING Maneuver", maneuverTask[0])            
            else:
                # Leave self.maneuverTaskCounter unchanged. Repeat this task.
                self.log("RUNNING Maneuver", maneuverTask[0], self.maneuverTaskCounter)
        elif (maneuverTask[0] == 'WHEEL_LOCK'):
            self.log("RUNNING AND ENDING Maneuver: Wheel Lock: ", self.maneuverTaskCounter)
            self.drivetrain.setWheelLock(True)
            self.drivetrain.goToPose(0, 0, self.drivetrain.getBearing())
            self.maneuverTaskCounter += 1 # Move on to next task
            self.elevator.update()
            self.grabber.update()
        elif (maneuverTask[0] == 'IDLE'):
            ## will run forever?
            self.log("RUNNING Maneuver: Idle: ", self.maneuverTaskCounter)
            self.drivetrain.idle()
            self.elevator.update()
            self.grabber.update()
        else:
            self.log("RUNNING Maneuver: ERROR: Unknown Task", self.maneuverTaskCounter, maneuverTask[0])
            self.maneuverTaskCounter += 1   
        return

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

    def log(self, *dataToLog):
        self.logger.log(MODULE_NAMES.ROBOT,dataToLog)

if __name__ == "__main__":
    if sys.argv[1] == 'sim':
        TEST_MODE = True
    wpilib.run(MyRobot)
