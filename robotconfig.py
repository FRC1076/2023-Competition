from collections import namedtuple

DEADZONE = 0.1

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

##########################
###  ROBOT COMPONENTS  ###
##########################
controllerConfig = {
    'DRIVER': {
        'ID': 0,
        'DEADZONE': DEADZONE,
        'LEFT_TRIGGER_AXIS': 2,
        'RIGHT_TRIGGER_AXIS': 3,
    },
    'OPERATOR': {
        'ID': 1,
        'DEADZONE': DEADZONE,
        'LEFT_TRIGGER_AXIS': 2,
        'RIGHT_TRIGGER_AXIS': 3,
    }
}

swervometerConfig = { # All positions measured in inches
    'TEAM_IS_RED': False, # Is the robot part of the Red Team?
    'FIELD_START_POSITION': 'B', # Which of three starting positions is selected?
    'HAS_BUMPERS_ATTACHED': True, # Does the robot currently have bumpers attached?
    'USE_COM_ADJUSTMENT': True, # Should robot compensate for CoM lever arms?
    'FIELD_ORIGIN_X': 0.0, # X-Coordinate of field orgin (center of field, viewed from scoring table)
    'FIELD_ORIGIN_Y': 0.0, # Y-Coordinate of field orgin (center of field, viewed from scoring table)
    'FIELD_RED_A_START_POSITION_X': 248.625, #159.0, # X-Coordinate of starting position A when on red team
    'FIELD_RED_A_START_POSITION_Y': 16.75, #40.15, #54.25, # Y-Coordinate of starting postion A when on red team
    'FIELD_RED_A_START_ANGLE': 0.0, # Heading angle of starting position A when on red team
    'FIELD_RED_B_START_POSITION_X': 248.625, # X-Coordinate of starting position B when on red team
    'FIELD_RED_B_START_POSITION_Y': -49.25, #-28.25, # Y-Coordinate of starting postion B when on red team
    'FIELD_RED_B_START_ANGLE': 0.0, # Heading angle of starting position B when on red team
    'FIELD_RED_C_START_POSITION_X': 248.625, # X-Coordinate of starting position C when on red team
    'FIELD_RED_C_START_POSITION_Y': -115.25, #-137.90, # Y-Coordinate of starting postion C when on red team
    'FIELD_RED_C_START_ANGLE': 0.0, # Heading angle of starting position C when on red team
    'FIELD_BLU_A_START_POSITION_X': -248.625, # X-Coordinate of starting position A when on blue team
    'FIELD_BLU_A_START_POSITION_Y': 16.75, # 40.15, # Y-Coordinate of starting postion A when on blue team
    'FIELD_BLU_A_START_ANGLE': 180.0, # Heading angle of starting position A when on blue team
    'FIELD_BLU_B_START_POSITION_X': -248.625, # X-Coordinate of starting position B when on blue team
    'FIELD_BLU_B_START_POSITION_Y': -49.25, #-28.25, # Y-Coordinate of starting postion B when on blue team
    'FIELD_BLU_B_START_ANGLE': 180.0, # Heading angle of starting position B when on blue team
    'FIELD_BLU_C_START_POSITION_X': -248.625, # X-Coordinate of starting position C when on blue team
    'FIELD_BLU_C_START_POSITION_Y': -115.25, # -137.90, # Y-Coordinate of starting postion C when on blue team
    'FIELD_BLU_C_START_ANGLE': 180.0, # Heading angle of starting position C when on blue team
    'ROBOT_FRAME_DIMENSION_X': 34.0, # X-coordinate length of robot frame
    'ROBOT_FRAME_DIMENSION_Y': 26.0, # Y-coordinate length of robot frame
    'ROBOT_BUMPER_DIMENSION_X': 3.0, # Width of bumper (X-axis)
    'ROBOT_BUMPER_DIMENSION_Y': 3.0, # Width of bumper (Y-axis)
    'ROBOT_COF_OFFSET_X': 17.0, # X-offset of center of frame (assume half frame dimension)
    'ROBOT_COF_OFFSET_Y': 13.0, # Y-offset of center of frame (assume half frame dimension)
    'ROBOT_COM_OFFSET_X': -0.75, # X-offset of center of mass (relative to center of frame)
    'ROBOT_COM_OFFSET_Y': -0.75, # Y-offset of center of mass (relative to center of frame)
    'ROBOT_GYRO_OFFSET_X': 15.0, # X-offset of center of gyro (relative to lower left frame)
    'ROBOT_GYRO_OFFSET_Y': 12.0, # Y-offset of center of gyro (relative to lower left frame)
    'ROBOT_CAMERA_OFFSET_X': 17.0, # X-offset of center of camera lens (relative to center of frame)
    'ROBOT_CAMERA_OFFSET_Y': 0.0, # Y-offset of center of camera lens (relative to center of frame)
    'ROBOT_CAMERA_HEIGHT': 12.1875, # Height of camera eye relative to gyroscope: 11 3/16+ 2 -1
    'ROBOT_SWERVE_MODULE_OFFSET_X': 13.75, # X-offset of swerve module center from COF
    'ROBOT_SWERVE_MODULE_OFFSET_Y': 9.75, # Y-offset of swerve module center from COF
}

drivetrainConfig = {
    'FRONTLEFT_DRIVEMOTOR': 1,
    'FRONTRIGHT_DRIVEMOTOR': 2,
    'REARRIGHT_DRIVEMOTOR': 3,
    'REARLEFT_DRIVEMOTOR': 4,
    'FRONTLEFT_ROTATEMOTOR': 11,
    'FRONTRIGHT_ROTATEMOTOR': 12,
    'REARRIGHT_ROTATEMOTOR': 13,
    'REARLEFT_ROTATEMOTOR': 14,
    'FRONTLEFT_ENCODER': 21,
    'FRONTRIGHT_ENCODER': 22,
    'REARRIGHT_ENCODER': 23,
    'REARLEFT_ENCODER': 24,
    'DRIVETYPE': SWERVE,
    'HEADING_KP': 0.005, #0.005 - reverted to this
    'HEADING_KI': 0.00001, #0.00001 - reverted to this
    'HEADING_KD':  0.00001, #0.00001 - reverted to this
    'BALANCE_PITCH_KP': 0.01, #0.02, #0.01
    'BALANCE_PITCH_KI': 0.00001, #0.01, #0.00001
    'BALANCE_PITCH_KD':  0.0005, #0.0001, #0.0005
    'BALANCE_YAW_KP': 0.005,
    'BALANCE_YAW_KI': 0.00001,
    'BALANCE_YAW_KD': 0.00001,
    'TARGET_KP': 0.025,
    'TARGET_KI': 0.001, #0.005,
    'TARGET_KD': 0.001,
    'BEARING_KP': 0.025,
    'BEARING_KI': 0.01,
    'BEARING_KD': 0.0001,
    'ROBOT_INCHES_PER_ROTATION': 1.0, #1.793, # Inches per rotation of wheels
    'TELEOP_OPEN_LOOP_RAMP_RATE': 0.125, # Improves maneuverability of bot.
    'TELEOP_CLOSED_LOOP_RAMP_RATE': 0.125,
    'AUTON_STEER_STRAIGHT': True,
    'TELEOP_STEER_STRAIGHT': False,
    'LOW_CONE_SCORE': [['CLAW_INTAKE'],
                        ['ELEVATOR_DOWN'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['CLAW_RELEASE'],
                        ['CLAW_STOP']],
    'HIGH_CONE_SCORE': [['CLAW_INTAKE'],
                        ['ELEVATOR_DOWN'],
                        ['POSITION_GRABBER', 2],
                        ['MOVE_BACK', 10],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE'],
                        ['CLAW_STOP']],
    'HUMAN_STATION_PICKUP': [['CLAW_INTAKE'],
                        ['ELEVATOR_DOWN'],
                        ['MOVE_BACK', 10],
                        ['POSITION_GRABBER', 2],
                        ['CLAW_RELEASE'],
                        ['ELEVATOR_HUMAN_EXTEND']],
    'ROTATE_CLOCKWISE': [['ROTATE', 179]], # 179, not -180 to ensure direction
    'ROTATE_COUNTERCLOCKWISE': [['ROTATE', -179]], # -179, not -180, to ensure direction
    'X_VISION_DRIVE_KP': 0.0050, # 0.005,
    'X_VISION_DRIVE_KI': 0.001, #0.0006, #0.0045, # 0.00001,
    'X_VISION_DRIVE_KD': 0.0, # 0.0005,
    'Y_VISION_DRIVE_KP': 0.0052, #0.005, # 0.0125,
    'Y_VISION_DRIVE_KI': 0.0001, #0.0006, #0.0045, #0.0, # 0.00001,
    'Y_VISION_DRIVE_KD': 0.0, # 0.0005,
    'REFLECTIVE_TARGET_TARGET_SIZE': 0.546, # 0.546% of the total field of view
    'REFLECTIVE_TARGET_OFFSET_X': -17.53,
    'APRIL_TARGET_TARGET_SIZE': 0.546, # % of the total field of view
    'APRIL_TARGET_OFFSET_X': -17.53,
    'MAX_TARGET_OFFSET_X': 90,
    'MIN_TARGET_SIZE': 0,
}

visionConfig = {
    'TARGET_HEIGHT': 8.5,
    'TARGET_RADIUS': 2,
    'SHOOTER_HEIGHT': 3.5,
    'SHOOTER_OFFSET': 1,
    'CAMERA_HEIGHT': 4,
    'CAMERA_PITCH': 0,
    'APRILTAGS': 0,
    'RETROREFLECTIVE': 1,
    'MIN_TARGET_ASPECT_RATIO_REFLECTIVE': 0.0,
    'MAX_TARGET_ASPECT_RATIO_REFLECTIVE': 100.0,
    'MIN_TARGET_ASPECT_RATIO_APRILTAG': 0.0,
    'MAX_TARGET_ASPECT_RATIO_APRILTAG': 100.0,
    'UPDATE_POSE': False, # True if should correct position with Limelight information. Otherwise informational.
}

elevatorConfig = {
    'RIGHT_ID': 6,
    'LEFT_ID': 5,
    'SOLENOID_FORWARD_ID': 15,
    'SOLENOID_REVERSE_ID': 14,
    'ELEVATOR_KP': 0.6, # 0.2, #0.8, #0.3, #0.48
    'ELEVATOR_KI': 0, #0.15, # 1.0, #0.0008, #0.0008, #0.0008
    'ELEVATOR_KD': 0.05, #0.01, # 0, #0.25, #0.2, #0.03
    'LOWER_SAFETY': 1,
    'UPPER_SAFETY': 33,
    'LEFT_LIMIT_SWITCH': 3, # Failsafe, hopefully one of them triggers
    'RIGHT_LIMIT_SWITCH': 4, # Failsafe, hopefully one of them triggers
    'CONE_ELEVATOR_HUMAN_POSITION': 22, # Assumes Elevator Down
    'CONE_ELEVATOR_UPPER_SCORING_HEIGHT': 29, #30.25, # Assumes Elevator Down
    'CONE_ELEVATOR_LOWER_SCORING_HEIGHT': 18.50, # Assumes Elevator Down
    'CONE_ELEVATOR_RETRACTED_HEIGHT': 7,
    'CUBE_ELEVATOR_HUMAN_POSITION': 21.0, # Assumes Elevator Down
    'CUBE_ELEVATOR_UPPER_SCORING_HEIGHT': 25.5, # Assumes Elevator Down
    'CUBE_ELEVATOR_LOWER_SCORING_HEIGHT': 15.25, # Assumes Elevator Down
    'CUBE_ELEVATOR_RETRACTED_HEIGHT': 7,
}

grabberConfig = {
    'ROTATE_MOTOR_ID': 7,
    'GRABBER_ROTATE_SPEED': 0.125,
    'GRABBER_KP': 0.1, # 0.1, #0.2, #0.12,3
    'GRABBER_KI': 0, # 1.0, #0.0008, #0.0008,
    'GRABBER_KD': 0, # 0.001, #0.002,
    'MAX_POSITION': 2.5, # Roughly 0 - 5 scale, with 0 at top
    'MIN_POSITION': 0.25, # Roughly 0 - 5 scale, with 0 at top
    'CONE_GRABBER_HUMAN_POSITION': 1.3, # Assumes Elevator Down
    'CONE_GRABBER_UPPER_SCORING_HEIGHT': 0.25, #Asssumes Elevator Down
    'CONE_GRABBER_LOWER_SCORING_HEIGHT': 0.5, # Assumes Elevator Down
    'CONE_GRABBER_RETRACTED_HEIGHT': 0.95,
    'CUBE_GRABBER_HUMAN_POSITION': 1.3, # Assumes Elevator Down
    'CUBE_GRABBER_UPPER_SCORING_HEIGHT': 0.8, #Asssumes Elevator Down
    'CUBE_GRABBER_LOWER_SCORING_HEIGHT': 0.8, # Assumes Elevator Down
    'CUBE_GRABBER_RETRACTED_HEIGHT': 1.3,}

clawConfig = {
    'MOTOR_ID': 8,
    'CONE_DEFAULT_RELEASE_SPEED': 0.125, #0.125, # Go slow on release, so piece drops straight down
    'CONE_UPPER_SCORING_HEIGHT_RELEASE_SPEED': 0.16,
    'CONE_LOWER_SCORING_HEIGHT_RELEASE_SPEED': 0.125,
    'CUBE_DEFAULT_RELEASE_SPEED': 0.125, #0.125, # Go slow on release, so piece drops straight down
    'CUBE_UPPER_SCORING_HEIGHT_RELEASE_SPEED': 0.18,
    'CUBE_LOWER_SCORING_HEIGHT_RELEASE_SPEED': 0.125,
    'RELEASE_CHANGE': 12, # Encoder change before we assume element is expelled
    'INTAKE_SPEED': 0.5, # Go fast on intake
    'INTAKE_CHANGE': 0.25 # Encoder change before we assume element is taken in
}

cliffDetectorConfig = {
    'LEFT_CLIFF_DETECTOR_PINGID': 0,
    'LEFT_CLIFF_DETECTOR_ECHOID': 1,
    'RIGHT_CLIFF_DETECTOR_PINGID': 2,
    'RIGHT_CLIFF_DETECTOR_ECHOID': 3,
    'CLIFF_TOLERANCE': 2, # Centimeters?
}

autonConfig = {
    'SCORE_EXISTING': True,
    'BALANCE_BOT': True,
    'DO_COMMUNITY': False, # Only applies for position B
    'AUTON_OPEN_LOOP_RAMP_RATE': 1, # Improves the quality of swervometery by avoiding slippage.
    'AUTON_CLOSED_LOOP_RAMP_RATE': 0,
    'TASK_BLU_A_TF': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, 40.15, 0],
                        ['IDLE']],
    'TASK_BLU_A_TT': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, 40.15, 0],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['MOVE_TO_BALANCE', -170, -28.25, 0, 10],
                        ['BALANCE']],
                        #['HALF_MOON_BALANCE', -161.875, 18, -161.875, 0, 180, 10]],
    'TASK_BLU_A_FT': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', -91.9375, 40.15, 0],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['MOVE_TO_BALANCE', -170, -28.25, 0, 10],
                        ['BALANCE']],
                        #['HALF_MOON_BALANCE', -161.875, 18, -161.875, 0, 180, 10]],
    'TASK_BLU_A_FF': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', -91.9375, 40.15, 0]],
    'TASK_BLU_B_TF': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['IDLE']],
    'TASK_BLU_B_TTT': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, -49.25, 0],
                        ['MOVE_TO_BALANCE', -170, -49.25, 0, 10],
                        ['BALANCE']],
    'TASK_BLU_B_TTF': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE_TO_BALANCE', -170, -49.25, 0, 10],
                        ['BALANCE']],
    'TASK_BLU_B_FTT': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', -91.9375, -49.25, 0],
                        ['MOVE', -170, -49.25, 0],
                        ['BALANCE']],
    'TASK_BLU_B_FTF': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', -170, -49.25, 0],
                        ['BALANCE']],
    'TASK_BLU_B_FF': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', -91.9375, -49.25, 0]],
    'TASK_BLU_C_TF': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        #['MOVE', -230.9375, -137.90, 0],
                        #['MOVE', -230.9375, -117.90, 0],
                        ['MOVE', -91.9375, -137.90, 0],
                        ['IDLE']],
    'TASK_BLU_C_TT': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, -137.90, 0],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['MOVE_TO_BALANCE', -170, -28.25, 0, 10],
                        ['BALANCE']],
                        #['HALF_MOON_BALANCE', -161.875, -114, -161.875, -96, 180, 10]],
    'TASK_BLU_C_FT': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', -91.9375, -137.90, 0],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['MOVE_TO_BALANCE', -170, -28.25, 0, 10],
                        ['BALANCE']],
                        #['HALF_MOON_BALANCE', -161.875, -114, -161.875, -96, 180, 10]],
    'TASK_BLU_C_FF': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', -91.9375, -137.90, 0]],
    'TASK_RED_A_TF': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', 91.9375, 40.15, 180],
                        ['IDLE']],
    'TASK_RED_A_TT': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', 91.9375, 40.15, 180],
                        ['MOVE', 91.9375, -28.25, 180],
                        ['MOVE_TO_BALANCE', 170, -28.25, 180, 10],
                        ['BALANCE']],
                        #['HALF_MOON_BALANCE', 161.875, 18, 161.875, 0, 180, 10]],
    'TASK_RED_A_FT': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', 91.9375, 40.15, 180],
                        ['MOVE', 91.9375, -28.25, 180],
                        ['MOVE_TO_BALANCE', 170, -28.25, 180, 10],
                        ['BALANCE']],
                        #['HALF_MOON_BALANCE', 161.875, 18, 161.875, 0, 180, 10]],
    'TASK_RED_A_FF': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', 91.9375, 40.15, 180]],
    'TASK_RED_B_TF': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', 91.9375, -49.25, 180],
                        ['IDLE']],
    'TASK_RED_B_TTT': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', 91.9375, -49.25, 180],
                        ['MOVE_TO_BALANCE', 170, -49.25, 180, 10],
                        ['BALANCE']],
    'TASK_RED_B_TTF': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE_TO_BALANCE', 170, -49.25, 180, 10],
                        ['BALANCE']],
    'TASK_RED_B_FTT': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', 91.9375, -49.25, 180],
                        ['MOVE_TO_BALANCE', 170, -49.25, 180, 10],
                        ['BALANCE']],
    'TASK_RED_B_FTF': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE_TO_BALANCE', 170, -49.25, 180, 10],
                        ['BALANCE']],
    'TASK_RED_B_FF': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', 91.9375, -49.25, 180]],
    'TASK_RED_C_TF': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_RETRACT'],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        #['MOVE', 200.9375, -137.90, 180],
                        #['MOVE', 150.9375, -137.90, 180],
                        ['MOVE', 91.9375, -137.90, 180],
                        ['IDLE']],
    'TASK_RED_C_TT': [['CLAW_INTAKE_AND_STOP'],
                        ['POSITION_GRABBER', 2],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['CLAW_RELEASE_AND_STOP'],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', 91.9375, -137.90, 180],
                        ['MOVE', 91.9375, -28.25, 180],
                        ['MOVE_TO_BALANCE', 170, -28.25, 180, 10],
                        ['BALANCE']],
                        #['HALF_MOON_BALANCE', 161.875, -114, 161.875, -96, 180, 10]],
    'TASK_RED_C_FT': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', 91.9375, -137.90, 180],
                        ['MOVE', 91.9375, -28.25, 180],
                        ['MOVE_TO_BALANCE', 170, -28.25, 180, 10],
                        ['BALANCE']],
                        #['HALF_MOON_BALANCE', 161.875, -114, 161.875, -96, 180, 10]],
    'TASK_RED_C_FF': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', 91.9375, -137.90, 180]],
}

#MODULE NAMES
MODULE_NAMES = namedtuple('MODULE_NAMES', [
    'ROBOT',
    'SWERVEDRIVE',
    'SWERVEMODULE',
    'SWERVOMETER',
    'ELEVATOR',
    'GRABBER',
    'CLAW',
    'VISION'
])

# using a named tuple to make sure we always use the same names
MODULE_NAMES.ROBOT = 'ROBOT'
MODULE_NAMES.SWERVEDRIVE = 'SWERVEDRIVE'
MODULE_NAMES.SWERVEMODULE = 'SWERVEMODULE'
MODULE_NAMES.SWERVOMETER = 'SWERVOMETER'
MODULE_NAMES.ELEVATOR = 'ELEVATOR'
MODULE_NAMES.GRABBER = 'GRABBER'
MODULE_NAMES.CLAW = 'CLAW'
MODULE_NAMES.VISION = 'VISION'


loggingConfig = {
    MODULE_NAMES.ROBOT: False,
    MODULE_NAMES.SWERVEDRIVE: False,
    MODULE_NAMES.SWERVEMODULE: False,
    MODULE_NAMES.SWERVOMETER: False,
    MODULE_NAMES.ELEVATOR: False,
    MODULE_NAMES.GRABBER: False,
    MODULE_NAMES.CLAW: False,
    MODULE_NAMES.VISION: False,    
}

dashboardConfig = {
    MODULE_NAMES.ROBOT: True,
    MODULE_NAMES.SWERVEDRIVE: True,
    MODULE_NAMES.SWERVEMODULE: True,
    MODULE_NAMES.SWERVOMETER: True,
    MODULE_NAMES.ELEVATOR: True,
    MODULE_NAMES.GRABBER: True,
    MODULE_NAMES.CLAW: True,
    MODULE_NAMES.VISION: True,   
}

#######################
###  ROBOT CONFIGS  ###
#######################
# testbot = { # Always used for unit tests ($ python robot.py sim)
#     'CONTROLLERS': controllerConfig,
#     'SWERVOMETER': swervometerConfig, # Must be BEFORE drivetrain
#     'VISION': visionConfig, # Must be BEFORE drivetrain
#     'DRIVETRAIN': drivetrainConfig,
#     'CLIFFDETECTOR': cliffDetectorConfig,
#     'GRABBER': grabberConfig, #MUST BE BEFORE ELEVATOR
#     'ELEVATOR': elevatorConfig,
#     'AUTON': autonConfig,
# }

showbot = {
    'CONTROLLERS': controllerConfig,
    'SWERVOMETER': swervometerConfig, # Must be BEFORE drivetrain
    'VISION': visionConfig, # Must be BEFORE drivetrain
    'DRIVETRAIN': drivetrainConfig,
    'CLAW': clawConfig,
    'GRABBER': grabberConfig, #MUST BE BEFORE ELEVATOR
    'ELEVATOR': elevatorConfig,
    'AUTON': autonConfig,
    'LOGGING': loggingConfig,
    'DASHBOARD': dashboardConfig
}

#showbot['DRIVETRAIN']['FRONTLEFT_DRIVEMOTOR'] = 1 # how to override just one thing

##########################
###  CONFIG TO DEPLOY  ###
##########################
robotconfig = showbot
