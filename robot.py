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
from swervedrive import BearingConfig
from swervometer import FieldConfig
from swervometer import RobotPropertyConfig
from swervometer import Swervometer
from cliffdetector import CliffDetector
from claw import Claw

from tester import Tester
from networktables import NetworkTables

from elevator import Elevator
from grabber import Grabber
from vision import Vision

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
        self.cliffDetector = None
        self.auton = None
        self.vision = None
        self.elevator = None
        self.grabber = None
        self.claw = None

        # Even if no drivetrain, defaults to drive phase
        self.phase = "DRIVE_PHASE"

        if TEST_MODE:
            self.config = Tester.getTestConfig()
        else:
            self.config = robotconfig

        NetworkTables.initialize(server='roborio-1076-frc.local') # Necessary for vision to
        self.dashboard = NetworkTables.getTable('SmartDashboard')

        print(self.config)
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

        if TEST_MODE:
            self.tester = Tester(self)
            self.tester.initTestTeleop()
            self.tester.testCodePaths()

    def disabledExit(self):
        print("no longer disabled")
        self.drivetrain.reset()

        # Reset task counter.
        self.autonTaskCounter = 0
        self.maneuverTaskCounter = 0

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
            self.team_is_red = True
            self.team_is_blu = False
            teamGyroAdjustment = 180 # Red Team faces 180 degrees at start.
            teamMoveAdjustment = -1 # Red Team needs to flip the controls as well.
        else:
            self.team_is_red = False
            self.team_is_blu = True
            teamGyroAdjustment = 0 # Blue Team faces 0 degrees at start.
            teamMoveAdjustment = 1 # Blue Team does not need to flip controlls.

        self.dashboard.putBoolean('Team is Red', self.team_is_red)

        print("FIELD_START_POSITION:", config['FIELD_START_POSITION'])

        if (config['FIELD_START_POSITION'] == 'A'):
            self.dashboard.putString('Field Start Position', 'A')
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
            self.dashboard.putString('Field Start Position', 'B')
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
            self.dashboard.putString('Field Start Position', 'C')
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
        vision = Vision(NetworkTables.getTable('limelight'), config['UPDATE_POSE'])

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
                            self.grabber)
        self.human_position = config['HUMAN_POSITION']
        self.upper_scoring_height = config['UPPER_SCORING_HEIGHT']
        self.lower_scoring_height = config['LOWER_SCORING_HEIGHT']
        self.retracted_height = config['RETRACTED_HEIGHT']
        self.elevator_is_automatic = False
        self.elevator_destination = 0
        return elevator

    def initGrabber(self, config):
        return Grabber(config['SUCTION_MOTOR_ID'], config['ROTATE_MOTOR_ID'], config['BOTTOM_SWITCH_ID'], config['TOP_SWITCH_ID'], config['GRABBER_ROTATE_SPEED'], config['GRABBER_SUCTION_SPEED'])

    def initClaw(self, config):
        return Claw(
                            config['SOLENOID_FORWARD_ID'], 
                            config['SOLENOID_REVERSE_ID']
        )
    
    def initDrivetrain(self, config):
        print("initDrivetrain ran")
        self.drive_type = config['DRIVETYPE']  # side effect!

        self.rotationCorrection = config['ROTATION_CORRECTION']

        balance_cfg = BalanceConfig(sd_prefix='Balance_Module', balance_pitch_kP=config['BALANCE_PITCH_KP'], balance_pitch_kI=config['BALANCE_PITCH_KI'], balance_pitch_kD=config['BALANCE_PITCH_KD'], balance_yaw_kP=config['BALANCE_YAW_KP'], balance_yaw_kI=config['BALANCE_YAW_KI'], balance_yaw_kD=config['BALANCE_YAW_KD'])
        target_cfg = TargetConfig(sd_prefix='Target_Module', target_kP=config['TARGET_KP'], target_kI=config['TARGET_KI'], target_kD=config['TARGET_KD'])
        bearing_cfg = BearingConfig(sd_prefix='Bearing_Module', bearing_kP=config['BEARING_KP'], bearing_kI=config['BEARING_KI'], bearing_kD=config['BEARING_KD'])

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

        self.lowConeScoreTaskList = config['LOW_CONE_SCORE']
        self.highConeScoreTaskList = config['HIGH_CONE_SCORE']
        self.humanStationTaskList = config['HUMAN_STATION_PICKUP']

        #gyro = AHRS.create_spi()
        gyro = AHRS.create_spi(wpilib._wpilib.SPI.Port.kMXP, 500000, 50) # https://www.chiefdelphi.com/t/navx2-disconnecting-reconnecting-intermittently-not-browning-out/425487/36
        
        swerve = SwerveDrive(rearLeftModule, frontLeftModule, rearRightModule, frontRightModule, self.swervometer, self.vision, gyro, balance_cfg, target_cfg, bearing_cfg)

        return swerve

    def initAuton(self, config):
        self.autonScoreExisting = config['SCORE_EXISTING']
        self.autonBalanceRobot = config['BALANCE_BOT']

        self.dashboard.putNumber('Auton Score Existing Element', self.autonScoreExisting)
        self.dashboard.putNumber('Auton Balance Robot', self.autonBalanceRobot)

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
            and self.fieldStartPosition == 'B'
            and self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_B_TF']     
        elif (self.team_is_red
            and self.fieldStartPosition == 'B'
            and self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_B_TT']     
        elif (self.team_is_red
            and self.fieldStartPosition == 'B'
            and not self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_RED_B_FT']     
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
            and self.fieldStartPosition == 'B'
            and self.autonScoreExisting
            and not self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_B_TF']
        elif (not self.team_is_red
            and self.fieldStartPosition == 'B'
            and self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_B_TT']
        elif (not self.team_is_red
            and self.fieldStartPosition == 'B'
            and not self.autonScoreExisting
            and self.autonBalanceRobot):
                self.autonTaskList = config['TASK_BLU_B_FT']
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
        else: # No matching task list
            self.autonTaskCounter = -1
            self.autonTaskList = []
        return True

    def initCliffDetector(self, config):
        print(config)
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
        #self.grabber.update()
        return True

    def teleopInit(self):
        print("teleopInit ran")

        self.drivetrain.setRampRates(self.teleopOpenLoopRampRate, self.teleopClosedLoopRampRate)
        self.grabber.engage()
        self.maneuverComplete = True
        self.startingManeuver = True
        self.maneuverTaskCounter = 0

        return True

    def teleopPeriodic(self):
        self.teleopDrivetrain()
        self.teleopElevator()
        self.teleopGrabber()
        return True

    def move(self, x, y, rcw):
        """
        This function is meant to be used by the teleOp.
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

        #if self.cliffdetector:
        #    if self.cliffdetector.atCliff() == -1:
        #        print("Warning: At Left Cliff!!!")
        #    elif self.cliffdetector.atCliff() == 1:
        #        print("Warning: At Right Cliff!!!")
        #    elif self.cliffdetector.atCliff() == 0:
        #        print("Coast is clear. Not near a cliff.")
        #    else:
        #        print("Bogus result from cliff detector. Ignore danger.")

        self.drivetrain.move(y, x, rcw, self.drivetrain.getBearing())
        #self.drivetrain.move(0, y, 0)

    def teleopDrivetrain(self):
        if (not self.drivetrain):
            return
        if (not self.driver):
            return

        driver = self.driver.xboxController
        deadzone = self.driver.deadzone

        speedMulti = 1.0

        self.dashboard.putNumber('ctrl right x', driver.getLeftX())
        self.dashboard.putNumber('ctrl right y', driver.getLeftY())
        #print("dashboard: ", self.dashboard.getNumber('ctrl right y', -1))
        
        # Note this is a bad idea in competition, since it's reset automatically in robotInit.
        if (driver.getLeftTriggerAxis() > 0.7 and driver.getRightTriggerAxis() > 0.7):
            self.drivetrain.resetGyro()
            self.drivetrain.printGyro()

        if (driver.getRightBumper()):
            speedMulti = 0.4

        #print("gyro yaw: " + str(self.drivetrain.getGyroAngle()))

        if (driver.getLeftBumper()):
            self.drivetrain.setWheelLock(True)
        else:
            self.drivetrain.setWheelLock(False)
        
        if(driver.getAButton()):
            self.drivetrain.balance()
        elif (driver.getBButton()):
            if(self.startingManeuver == True):
                print("B Button - Starting Maneuver")
                self.startingManeuver = False
                self.maneuverComplete = False
                self.maneuverTaskCounter = 0
                self.maneuverTaskList = self.lowConeScoreTaskList
            self.teleopManeuver()
        elif (driver.getYButton()):
            if(self.startingManeuver == True):
                print("Y Button - Starting Maneuver")
                self.startingManeuver = False
                self.maneuverComplete = False
                self.maneuverTaskCounter = 0
                self.maneuverTaskList = self.highConeScoreTaskList
            self.teleopManeuver()
        elif (driver.getXButton()):
            if(self.startingManeuver == True):
                print("X Button - Starting Maneuver")
                self.startingManeuver = False
                self.maneuverComplete = False
                self.maneuverTaskCounter = 0
                self.maneuverTaskList = self.humanStationTaskList
            self.teleopManeuver()
        elif (driver.getBButton == False and driver.getYButton == False and self.maneuverComplete == True):
            self.startingManeuver = True
        else:
            rightXCorrected = self.deadzoneCorrection(-driver.getLeftX() * speedMulti, 0.30)
            rightYCorrected = self.deadzoneCorrection(driver.getLeftY() * speedMulti, 0.30)
            leftXCorrected = self.deadzoneCorrection(driver.getRightX() * speedMulti, 0.30)
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

    def teleopElevator(self):
        operator = self.operator.xboxController

        if (operator.getLeftBumperPressed()):
            print("Toggling Elevator")
            self.elevator.toggle()

        ## ignored for now
        clutch_factor = 1
        #Check for clutch
        if(operator.getLeftTriggerAxis() > 0.7):
            clutch_factor = 0.4
        #Find the value the arm will move at
        extend_value = (self.deadzoneCorrection(operator.getLeftY(), self.operator.deadzone) / 5) * clutch_factor
        #preset destinations
        if operator.getAButton(): #Lowest
            self.elevator_destination = self.retracted_height
            self.elevator_is_automatic = True
            #print("Elevator: A Button")
        if operator.getYButton() and self.elevator.isElevatorDown(): #Highest
            self.elevator_destination = self.upper_scoring_height
            self.elevator_is_automatic = True
            #print("Elevator: Y Button")
        if operator.getBButton() and self.elevator.isElevatorDown(): #Medium Position
            self.elevator_destination = self.lower_scoring_height
            self.elevator_is_automatic = True
            #print("Elevator: B Button")
        if operator.getXButton(): #Human Position
            self.elevator_destination = self.human_position
            self.elevator_is_automatic = True
            #print("Elevator: X Button")

        #if controller is moving, disable elevator automatic move
        if(abs(extend_value) > 0):
            self.elevator_is_automatic = False
        #if automatic move, move to destination position
        if self.elevator_is_automatic:
            print("Elevator move to Pos")
            self.elevator.moveToPos(self.elevator_destination)
        else:
            self.elevator.extend(extend_value)
        
    def teleopGrabber(self):
        operator = self.operator.xboxController
        # if the operator is holding the bumper, keep the grab going. Otherwise release.
        

        grabber_speed = (self.deadzoneCorrection(operator.getRightY(), self.operator.deadzone))

        if (grabber_speed > 0):
            print("Grabber: Raise Grabber")
            self.grabber.lower_motor(-grabber_speed)
        elif (grabber_speed < 0):
            print("Grabber: Lower Grabber")
            self.grabber.raise_motor(-grabber_speed)
        else:
            print("Grabber: Motor Off")
            self.grabber.motor_off()
        
        if (operator.getRightTriggerAxis() > 0.7):
            print("Grabber: Release Suction")
            self.grabber.release()
        else:
            print("Grabber: Engage Suction")
            self.grabber.engage()
        
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

        if self.autonTaskCounter < 0:
            return # No tasks assigned.

        if self.autonTaskCounter >= len(self.autonTaskList):
            return # No tasks remaining.
            
        autonTask = self.autonTaskList[self.autonTaskCounter]

        if (autonTask[0] == 'TIMER'):
            print("Auton: Timer: ", self.autonTimer.get())
            if self.autonTimer.get() > autonTask[1]:
                self.autonTaskCounter += 1
        elif (autonTask[0] == 'GRAB'):
            print("Auton: Grab: ", self.autonTaskCounter)
            self.grabber.engage()
            self.autonTaskCounter += 1
        elif (autonTask[0] == 'RELEASE'):
            print("Auton: Release: ", self.autonTaskCounter)
            self.grabber.release()
            self.autonTaskCounter += 1
        elif (autonTask[0] == 'RAISE_GRABBER'):
            if self.grabber.raise_motor(0.6):
                self.autonTaskCounter += 1
        elif (autonTask[0] == 'LOWER_GRABBER'):
            if self.grabber.lower_motor(0.2):
                self.autonTaskCounter += 1
        elif (autonTask[0] == 'LOWER_GRABBER_UNCHECKED'):
            self.grabber.lower_motor(0.2)
            self.autonTaskCounter += 1
        elif (autonTask[0] == 'ELEVATOR_TOGGLE'):
            if self.elevator.toggle():
                self.autonTaskCounter += 1
            print("Auton: Elevator Toggle: ", self.elevator.getEncoderPosition())
        elif (autonTask[0] == 'ELEVATOR_UP'):
            if self.elevator.elevatorUp():
                self.autonTaskCounter += 1
            print("Auton: Elevator Up: ", self.elevator.getEncoderPosition())
        elif (autonTask[0] == 'ELEVATOR_DOWN'):
            if self.elevator.elevatorDown():
                self.autonTaskCounter += 1
            print("Auton: Elevator Down: ", self.elevator.getEncoderPosition())
        elif (autonTask[0] == 'ELEVATOR_EXTEND'):
            if self.elevator.moveToPos(self.lower_scoring_height):
                self.autonTaskCounter += 1
            print("Auton: Elevator Extend: ", self.elevator.getEncoderPosition())
        elif (autonTask[0] == 'ELEVATOR_RETRACT'):
            if self.elevator.moveToPos(self.retracted_height):
                self.autonTaskCounter += 1
            print("Auton: Elevator Retract: ", self.elevator.getEncoderPosition())
        elif (autonTask[0] == 'MOVE'):
            x = autonTask[1]
            y = autonTask[2]
            bearing = autonTask[3]
            print("Auton: Move: ", self.autonTaskCounter, " Target: x: ", x, " y: ", y, " bearing: ", bearing)
            if self.drivetrain.goToPose(x, y, bearing):
                self.autonTaskCounter += 1 # Move on to next task.
                print("Auton: Move: Reached target: x: ", x, " y: ", y, " bearing: ", bearing)
            else:
                # Leave self.autonTaskCounter unchanged. Repeat this task.
                print("Auton: Move: Not at target: x: ", x, " y: ", y, " bearing: ", bearing)
        elif (autonTask[0] == 'BALANCE'):
            print("Auton: Balance: ", self.autonTaskCounter)
            if self.drivetrain.balance():
                self.autonTaskCounter += 1 # Move on to next task.
                print("Auton: Balance: Leveled and oriented")
            else:
                # Leave self.autonTaskCounter unchanged. Repeat this task.
                print("Auton: Balance: Keep balancing and orienting")
        elif (autonTask[0] == 'WHEEL_LOCK'):
            print("Auton: Wheel Lock: ", self.autonTaskCounter)
            self.drivetrain.setWheelLock(True)
            self.drivetrain.goToPose(0, 0, self.drivetrain.getBearing())
            self.autonTaskCounter += 1 # Move on to next task.
        elif (autonTask[0] == 'IDLE'):
            print("Auton: Idle: ", self.autonTaskCounter)
            self.drivetrain.idle()
        else:
            print("Auton: ERROR: Unknown Task", self.autonTaskCounter)
            self.autonTaskCounter += 1   

        return

    def teleopManeuver(self):
        if not self.drivetrain:
            return
        if not self.swervometer:
            return
        
        print("teleopManeuver")

        if self.maneuverTaskCounter < 0:
            return # No tasks assigned.

        if self.maneuverTaskCounter >= len(self.maneuverTaskList):
            self.maneuverComplete = True
            self.startingManeuver = True
            return # No tasks remaining.
            
        maneuverTask = self.maneuverTaskList[self.maneuverTaskCounter]

        print("WHICH TASK: ", maneuverTask[0])

        if (maneuverTask[0] == 'GRAB'):
            print("maneuver: Grab: ", self.maneuverTaskCounter)
            self.grabber.engage()
            self.maneuverTaskCounter += 1
        elif (maneuverTask[0] == 'RELEASE'):
            print("maneuver: Release: ", self.maneuverTaskCounter)
            self.grabber.release()
            self.maneuverTaskCounter += 1
        elif (maneuverTask[0] == 'RAISE_GRABBER'):
            print("maneuver: Rase Grabber: ", self.maneuverTaskCounter)
            if self.grabber.raise_motor(0.6):
                self.maneuverTaskCounter += 1
        elif (maneuverTask[0] == 'LOWER_GRABBER'):
            if self.grabber.lower_motor(0.2):
                self.maneuverTaskCounter += 1
        elif (maneuverTask[0] == 'LOWER_GRABBER_UNCHECKED'):
            self.grabber.lower_motor(0.2)
            self.maneuverTaskCounter += 1
        elif (maneuverTask[0] == 'ELEVATOR_TOGGLE'):
            if self.elevator.toggle():
                self.maneuverTaskCounter += 1
            print("maneuver: Elevator Toggle: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'ELEVATOR_UP'):
            if self.elevator.elevatorUp():
                self.maneuverTaskCounter += 1
            print("maneuver: Elevator Up: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'ELEVATOR_DOWN'):
            if self.elevator.elevatorDown():
                self.maneuverTaskCounter += 1
            print("maneuver: Elevator Down: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'ELEVATOR_LOWER_EXTEND'):
            if self.elevator.moveToPos(self.lower_scoring_height):
                self.maneuverTaskCounter += 1
            print("maneuver: Elevator Extend: ", self.lower_scoring_height, " current position: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'ELEVATOR_HUMAN_EXTEND'):
            if self.elevator.moveToPos(self.human_position):
                self.maneuverTaskCounter += 1
            print("maneuver: Elevator Extend: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'ELEVATOR_UPPER_EXTEND'):
            if self.elevator.moveToPos(self.upper_scoring_height):
                self.maneuverTaskCounter += 1
            print("maneuver: Elevator Extend: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'ELEVATOR_RETRACT'):
            if self.elevator.moveToPos(self.retracted_height):
                self.maneuverTaskCounter += 1
            print("maneuver: Elevator Retract: ", self.elevator.getEncoderPosition())
        elif (maneuverTask[0] == 'MOVE'):
            x = maneuverTask[1]
            y = maneuverTask[2]
            bearing = maneuverTask[3]
            print("maneuver: Move: ", self.maneuverTaskCounter, " Target: x: ", x, " y: ", y, " bearing: ", bearing)
            if self.drivetrain.goToPose(x, y, bearing):
                self.maneuverTaskCounter += 1 # Move on to next task.
                print("maneuver: Move: Reached target: x: ", x, " y: ", y, " bearing: ", bearing)
            else:
                # Leave self.maneuverTaskCounter unchanged. Repeat this task.
                print("maneuver: Move: Not at target: x: ", x, " y: ", y, " bearing: ", bearing)
        elif (maneuverTask[0] == 'MOVE_BACK'):
            backDistance = maneuverTask[1]
            x,y,bearing = self.swervometer.getCOF()
            if(self.team_is_red):
                x -= backDistance
            else:
                x += backDistance
            if self.drivetrain.goToPose(x, y, bearing):
                self.maneuverTaskCounter += 1 # Move on to next task.
                print("maneuver: Move: Reached target: x: ", x, " y: ", y, " bearing: ", bearing)
            else:
                # Leave self.maneuverTaskCounter unchanged. Repeat this task.
                print("maneuver: Move: Not at target: x: ", x, " y: ", y, " bearing: ", bearing)
        elif (maneuverTask[0] == 'BALANCE'):
            print("maneuver: Balance: ", self.maneuverTaskCounter)
            if self.drivetrain.balance():
                self.maneuverTaskCounter += 1 # Move on to next task.
                print("maneuver: Balance: Leveled and oriented")
            else:
                # Leave self.maneuverTaskCounter unchanged. Repeat this task.
                print("maneuver: Balance: Keep balancing and orienting")
        elif (maneuverTask[0] == 'WHEEL_LOCK'):
            print("maneuver: Wheel Lock: ", self.maneuverTaskCounter)
            self.drivetrain.setWheelLock(True)
            self.drivetrain.goToPose(0, 0, self.drivetrain.getBearing())
            self.maneuverTaskCounter += 1 # Move on to next task.
        elif (maneuverTask[0] == 'IDLE'):
            print("maneuver: Idle: ", self.maneuverTaskCounter)
            self.drivetrain.idle()
        else:
            print("maneuver: ERROR: Unknown Task", self.maneuverTaskCounter)
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

    def logResult(self, *result):
        if (TEST_MODE):
            print(result)


if __name__ == "__main__":
    #if sys.argv[1] == 'sim':
    #    TEST_MODE = True
    wpilib.run(MyRobot)
