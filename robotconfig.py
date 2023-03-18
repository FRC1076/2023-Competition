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
    'USE_COM_ADJUSTMENT': False, # Should robot compensate for CoM lever arms?
    'FIELD_ORIGIN_X': 0.0, # X-Coordinate of field orgin (center of field, viewed from scoring table)
    'FIELD_ORIGIN_Y': 0.0, # Y-Coordinate of field orgin (center of field, viewed from scoring table)
    'FIELD_RED_A_START_POSITION_X': 248.625, #159.0, # X-Coordinate of starting position A when on red team
    'FIELD_RED_A_START_POSITION_Y': 40.15, #54.25, # Y-Coordinate of starting postion A when on red team
    'FIELD_RED_A_START_ANGLE': 0.0, # Heading angle of starting position A when on red team
    'FIELD_RED_B_START_POSITION_X': 248.625, # X-Coordinate of starting position B when on red team
    'FIELD_RED_B_START_POSITION_Y': -28.25, # Y-Coordinate of starting postion B when on red team
    'FIELD_RED_B_START_ANGLE': 0.0, # Heading angle of starting position B when on red team
    'FIELD_RED_C_START_POSITION_X': 248.625, # X-Coordinate of starting position C when on red team
    'FIELD_RED_C_START_POSITION_Y': -137.90, # Y-Coordinate of starting postion C when on red team
    'FIELD_RED_C_START_ANGLE': 0.0, # Heading angle of starting position C when on red team
    'FIELD_BLU_A_START_POSITION_X': -248.625, # X-Coordinate of starting position A when on blue team
    'FIELD_BLU_A_START_POSITION_Y': 40.15, # Y-Coordinate of starting postion A when on blue team
    'FIELD_BLU_A_START_ANGLE': 180.0, # Heading angle of starting position A when on blue team
    'FIELD_BLU_B_START_POSITION_X': -248.625, # X-Coordinate of starting position B when on blue team
    'FIELD_BLU_B_START_POSITION_Y': -28.25, # Y-Coordinate of starting postion B when on blue team
    'FIELD_BLU_B_START_ANGLE': 180.0, # Heading angle of starting position B when on blue team
    'FIELD_BLU_C_START_POSITION_X': -248.625, # X-Coordinate of starting position C when on blue team
    'FIELD_BLU_C_START_POSITION_Y': -137.90, # Y-Coordinate of starting postion C when on blue team
    'FIELD_BLU_C_START_ANGLE': 180.0, # Heading angle of starting position C when on blue team
    'ROBOT_FRAME_DIMENSION_X': 34.0, # X-coordinate length of robot frame
    'ROBOT_FRAME_DIMENSION_Y': 26.0, # Y-coordinate length of robot frame
    'ROBOT_BUMPER_DIMENSION_X': 3.0, # Width of bumper (X-axis)
    'ROBOT_BUMPER_DIMENSION_Y': 3.0, # Width of bumper (Y-axis)
    'ROBOT_COF_OFFSET_X': 17.0, # X-offset of center of frame (assume half frame dimension)
    'ROBOT_COF_OFFSET_Y': 13.0, # Y-offset of center of frame (assume half frame dimension)
    'ROBOT_COM_OFFSET_X': -3.5, #-4.0 X-offset of center of mass (relative to center of frame)
    'ROBOT_COM_OFFSET_Y': 0.0, # Y-offset of center of mass (relative to center of frame)
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
    'BALANCE_PITCH_KP': 0.01,
    'BALANCE_PITCH_KI': 0.00001,
    'BALANCE_PITCH_KD':  0.0005,
    'BALANCE_YAW_KP': 0.005,
    'BALANCE_YAW_KI': 0.00001,
    'BALANCE_YAW_KD': 0.00001,
    'TARGET_KP': 0.005,
    'TARGET_KI': 0.005,
    'TARGET_KD': 0.0001,
    'BEARING_KP': 0.025,
    'BEARING_KI': 0.00001,
    'BEARING_KD': 0.0001,
    'ROBOT_INCHES_PER_ROTATION': 1.0, #1.793, # Inches per rotation of wheels
    'TELEOP_OPEN_LOOP_RAMP_RATE': 1.0, # Improves maneuverability of bot.
    'TELEOP_CLOSED_LOOP_RAMP_RATE': 1.0,
    'LOW_CONE_SCORE': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_UP'],
                        ['ELEVATOR_LOWER_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['CLAW_OPEN']],
    'HIGH_CONE_SCORE': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['MOVE_BACK', 6],
                        ['ELEVATOR_UPPER_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['CLAW_OPEN'],
                        ['LOWER_GRABBER']],
    'HUMAN_STATION_PICKUP': [['CLAW_OPEN'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_DOWN'],
                        ['ELEVATOR_HUMAN_EXTEND']],
    'ROTATE_CLOCKWISE': [['ROTATE', 179]], # 179, not -180 to ensure direction
    'ROTATE_COUNTERCLOCKWISE': [['ROTATE', -179]], # -179, not -180, to ensure direction
    'TARGET_TARGET_SIZE': 0.1, # Needs to be figured out
    'TARGET_OFFSET_X': 0, # Needs to be figured out
}

visionConfig = {
    'TARGET_HEIGHT': 8.5,
    'TARGET_RADIUS': 2,
    'SHOOTER_HEIGHT': 3.5,
    'SHOOTER_OFFSET': 1,
    'CAMERA_HEIGHT': 4,
    'CAMERA_PITCH': 0,
    'UPDATE_POSE': False, # True if should correct position with Limelight information. Otherwise informational.
}

elevatorConfig = {
    'RIGHT_ID': 6,
    'LEFT_ID': 5,
    'SOLENOID_FORWARD_ID': 15,
    'SOLENOID_REVERSE_ID': 14,
    'ELEVATOR_KP': 0.12, #0.048, # 0.8 * 0.6
    'ELEVATOR_KI': 0.0008, # 0.0525, # 2 * 0.048 / 1.62
    'ELEVATOR_KD': 0.02, # 0.00972, # 0.048 * 1.62 / 8
    'HUMAN_POSITION': 28, # Assumes Elevator Down
    'UPPER_SCORING_HEIGHT': 33,
    'LOWER_SCORING_HEIGHT': 17, # Assumes Elevator Down
    'RETRACTED_HEIGHT': 7,
    'LOWER_SAFETY': 15,
    'UPPER_SAFETY': 25,
    'LEFT_LIMIT_SWITCH': 3, # Failsafe, hopefully one of them triggers
    'RIGHT_LIMIT_SWITCH': 4, # Failsafe, hopefully one of them triggers
}

grabberConfig = {
    'SUCTION_MOTOR_ID': 7,
    'ROTATE_MOTOR_ID': 8,
    'TOP_SWITCH_ID': 1,
    'BOTTOM_SWITCH_ID': 0,
    'GRABBER_ROTATE_SPEED': 0.2,
    'GRABBER_SUCTION_SPEED': 0.25,
}

clawConfig = {
    'MOTOR_ID': 7,
    'RELEASE_SPEED': 0.1, # Go slow on release, so piece drops straight down
    'RELEASE_CHANGE': 100, # Encoder change before we assume element is grabbed
    'INTAKE_SPEED': 0.1, # Go fast on intake
    'INTAKE_CHANGE': 100 # Encoder change before we assume element is expelled
}

cliffDetectorConfig = {
    'LEFT_CLIFF_DETECTOR_PINGID': 0,
    'LEFT_CLIFF_DETECTOR_ECHOID': 1,
    'RIGHT_CLIFF_DETECTOR_PINGID': 2,
    'RIGHT_CLIFF_DETECTOR_ECHOID': 3,
    'CLIFF_TOLERANCE': 2, # Centimeters?
}

autonConfig = {
    'SCORE_EXISTING': False,
    'BALANCE_BOT': True,
    'AUTON_OPEN_LOOP_RAMP_RATE': 1, # Improves the quality of swervometery by avoiding slippage.
    'AUTON_CLOSED_LOOP_RAMP_RATE': 0,
    'TASK_BLU_A_TF': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, 40.15, 0],
                        ['IDLE']],
    'TASK_BLU_A_TT': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, 40.15, 0],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['MOVE_TO_BALANCE', -170, -28.25, 0, 10],
                        ['BALANCE']],
    'TASK_BLU_A_FT': [  ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', -91.9375, 40.15, 0],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['MOVE_TO_BALANCE', -170, -28.25, 0, 10],
                        ['BALANCE']],
    'TASK_BLU_B_TF': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['IDLE']],
    'TASK_BLU_B_TT': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['MOVE_TO_BALANCE', -170, -28.25, 0, 10],
                        ['BALANCE']],
    'TASK_BLU_B_FT': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['MOVE', -170, -28.25, 0],
                        ['BALANCE']],
    'TASK_BLU_C_TF': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, -137.90, 0],
                        ['IDLE']],
    'TASK_BLU_C_TT': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', -91.9375, -137.90, 0],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['MOVE_TO_BALANCE', -170, -28.25, 0, 10],
                        ['BALANCE']],
    'TASK_BLU_C_FT': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', -91.9375, -137.90, 0],
                        ['MOVE', -91.9375, -28.25, 0],
                        ['MOVE_TO_BALANCE', -170, -28.25, 0, 10],
                        ['BALANCE']],
    'TASK_RED_A_TF': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', 91.9375, 40.15, 180],
                        ['IDLE']],
    'TASK_RED_A_TT': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', 91.9375, 40.15, 180],
                        ['MOVE', 91.9375, -28.25, 180],
                        ['MOVE_TO_BALANCE', 170, -28.25, 180, 10],
                        ['BALANCE']],
    'TASK_RED_A_FT': [['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', 91.9375, 40.15, 180],
                        ['MOVE', 91.9375, -28.25, 180],
                        ['MOVE_TO_BALANCE', 170, -28.25, 180, 10],
                        ['BALANCE']],
    'TASK_RED_B_TF': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', 91.9375, -28.25, 180],
                        ['IDLE']],
    'TASK_RED_B_TT': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE_TO_BALANCE', 170, -28.25, 180, 10],
                        ['BALANCE']],
    'TASK_RED_B_FT': [  ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', 91.9375, -28.25, 180],
                        ['MOVE_TO_BALANCE', 170, -28.25, 180, 10],
                        ['BALANCE']],
    'TASK_RED_C_TF': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', 91.9375, -137.90, 180],
                        ['IDLE']],
    'TASK_RED_C_TT': [['CLAW_CLOSE'],
                        ['RAISE_GRABBER'],
                        ['ELEVATOR_EXTEND'],
                        ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['CLAW_OPEN'],
                        ['TIMER', 6.0],
                        ['ELEVATOR_RETRACT'],
                        ['MOVE', 91.9375, -137.90, 180],
                        ['MOVE', 91.9375, -28.25, 180],
                        ['MOVE_TO_BALANCE', 170, -28.25, 180, 10],
                        ['BALANCE']],
    'TASK_RED_C_FT': [  ['ELEVATOR_DOWN'],
                        ['TIMER', 2.0],
                        ['MOVE', 91.9375, -137.90, 180],
                        ['MOVE', 91.9375, -28.25, 180],
                        ['MOVE_TO_BALANCE', 170, -28.25, 180, 10],
                        ['BALANCE']],
}

#######################
###  ROBOT CONFIGS  ###
#######################
testbot = { # Always used for unit tests ($ python robot.py sim)
    'CONTROLLERS': controllerConfig,
    'SWERVOMETER': swervometerConfig, # Must be BEFORE drivetrain
    'VISION': visionConfig, # Must be BEFORE drivetrain
    'DRIVETRAIN': drivetrainConfig,
    'CLIFFDETECTOR': cliffDetectorConfig,
    'GRABBER': grabberConfig, #MUST BE BEFORE ELEVATOR
    'ELEVATOR': elevatorConfig,
    'AUTON': autonConfig,
}

showbot = {
    'CONTROLLERS': controllerConfig,
    'SWERVOMETER': swervometerConfig, # Must be BEFORE drivetrain
    'VISION': visionConfig, # Must be BEFORE drivetrain
    'DRIVETRAIN': drivetrainConfig,
    'CLAW': clawConfig,
    'GRABBER': grabberConfig, #MUST BE BEFORE ELEVATOR
    'ELEVATOR': elevatorConfig,
    'AUTON': autonConfig,
}

#showbot['DRIVETRAIN']['FRONTLEFT_DRIVEMOTOR'] = 1 # how to override just one thing

##########################
###  CONFIG TO DEPLOY  ###
##########################
robotconfig = showbot
