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
    'FIELD_ORIGIN_X': 0.0, # X-Coordinate of field orgin (center of field, viewed from scoring table)
    'FIELD_ORIGIN_Y': 0.0, # Y-Coordinate of field orgin (center of field, viewed from scoring table)
    'FIELD_RED_A_START_POSITION_X': 231, #159.0, # X-Coordinate of starting position A when on red team
    'FIELD_RED_A_START_POSITION_Y': 22.4, #54.25, # Y-Coordinate of starting postion A when on red team
    'FIELD_RED_A_START_ANGLE': 0.0, # Heading angle of starting position A when on red team
    'FIELD_RED_B_START_POSITION_X': 231, # X-Coordinate of starting position B when on red team
    'FIELD_RED_B_START_POSITION_Y': -65.7, # Y-Coordinate of starting postion B when on red team
    'FIELD_RED_B_START_ANGLE': 0.0, # Heading angle of starting position B when on red team
    'FIELD_RED_C_START_POSITION_X': 231, # X-Coordinate of starting position C when on red team
    'FIELD_RED_C_START_POSITION_Y': -137.7, # Y-Coordinate of starting postion C when on red team
    'FIELD_RED_C_START_ANGLE': 0.0, # Heading angle of starting position C when on red team
    'FIELD_BLU_A_START_POSITION_X': -231, # X-Coordinate of starting position A when on blue team
    'FIELD_BLU_A_START_POSITION_Y': 22.4, # Y-Coordinate of starting postion A when on blue team
    'FIELD_BLU_A_START_ANGLE': 180.0, # Heading angle of starting position A when on blue team
    'FIELD_BLU_B_START_POSITION_X': -231, # X-Coordinate of starting position B when on blue team
    'FIELD_BLU_B_START_POSITION_Y': -65.7, # Y-Coordinate of starting postion B when on blue team
    'FIELD_BLU_B_START_ANGLE': 180.0, # Heading angle of starting position B when on blue team
    'FIELD_BLU_C_START_POSITION_X': -231, # X-Coordinate of starting position C when on blue team
    'FIELD_BLU_C_START_POSITION_Y': -137.7, # Y-Coordinate of starting postion C when on blue team
    'FIELD_BLU_C_START_ANGLE': 180.0, # Heading angle of starting position C when on blue team
    'FIELD_TAG1_X': 500.0, # X-Coordinate of Tag ID 1
    'FIELD_TAG1_Y': 500.0, # Y-Coordinate of Tag ID 1
    'FIELD_TAG2_X': 500.0, # X-Coordinate of Tag ID 2
    'FIELD_TAG2_Y': 500.0, # Y-Coordinate of Tag ID 2
    'FIELD_TAG3_X': 500.0, # X-Coordinate of Tag ID 3
    'FIELD_TAG3_Y': 500.0, # Y-Coordinate of Tag ID 3
    'FIELD_TAG4_X': 500.0, # X-Coordinate of Tag ID 4
    'FIELD_TAG4_Y': 500.0, # Y-Coordinate of Tag ID 4
    'FIELD_TAG5_X': 500.0, # X-Coordinate of Tag ID 5
    'FIELD_TAG5_Y': 500.0, # Y-Coordinate of Tag ID 5
    'FIELD_TAG6_X': 500.0, # X-Coordinate of Tag ID 6
    'FIELD_TAG6_Y': 500.0, # Y-Coordinate of Tag ID 6
    'FIELD_TAG7_X': 500.0, # X-Coordinate of Tag ID 7
    'FIELD_TAG7_Y': 500.0, # Y-Coordinate of Tag ID 7
    'FIELD_TAG8_X': 500.0, # X-Coordinate of Tag ID 8
    'FIELD_TAG8_Y': 500.0, # Y-Coordinate of Tag ID 8
    'ROBOT_FRAME_DIMENSION_X': 34.0, # X-coordinate length of robot frame
    'ROBOT_FRAME_DIMENSION_Y': 26.0, # Y-coordinate length of robot frame
    'ROBOT_BUMPER_DIMENSION_X': 3.0, # Width of bumper (X-axis)
    'ROBOT_BUMPER_DIMENSION_Y': 3.0, # Width of bumper (Y-axis)
    'ROBOT_COF_OFFSET_X': 17.0, # X-offset of center of frame (assume half frame dimension)
    'ROBOT_COF_OFFSET_Y': 13.0, # Y-offset of center of frame (assume half frame dimension)
    'ROBOT_GYRO_OFFSET_X': 15.0, # X-offset of center of gyro (relative to lower left frame)
    'ROBOT_GYRO_OFFSET_Y': 12.0, # Y-offset of center of gyro (relative to lower left frame)
    'ROBOT_CAMERA_OFFSET_X': 17.0, # X-offset of center of camera lens (relative to lower left frame)
    'ROBOT_CAMERA_OFFSET_Y': 0.0, # Y-offset of center of camera lens (relative to lower left frame)
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
    'ROTATION_CORRECTION': 0.0,
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
    'ROBOT_INCHES_PER_ROTATION': 1.0, #1.793, # Inches per rotation of wheels
    'TELEOP_OPEN_LOOP_RAMP_RATE': 0.0, # Improves maneuverability of bot.
}

intakeConfig = {
    # Update IDs when known
    'INTAKE_MOTOR_ID': -1,
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

cliffDetectorConfig = {
    'LEFT_CLIFF_DETECTOR_PINGID': 0,
    'LEFT_CLIFF_DETECTOR_ECHOID': 1,
    'RIGHT_CLIFF_DETECTOR_PINGID': 2,
    'RIGHT_CLIFF_DETECTOR_ECHOID': 3,
    'CLIFF_TOLERANCE': 2, # Centimeters?
}

grabberConfig = {
    'RIGHT_ID': -1,
    'LEFT_ID': -2,
    'INTAKE_TOP_ID': -3,
    'INTAKE_BOTTOM_ID': -4,
    'SOLENOID_FORWARD_ID': -5,
    'SOLENOID_REVERSE_ID': -6
}

autonConfig = {
    'SCORE_EXISTING': True,
    'PICKUP_NEW': False,
    'SCORE_NEW': False,
    'BALANCE_BOT': True,
    'AUTON_OPEN_LOOP_RAMP_RATE': 1, # Improves the quality of swervometery by avoiding slippage.
    'TASK_BLU_A_TFFT': [['ELEVATE'],
                        ['MOVE', -122.6, 27.8, 180],
                        ['MOVE', -238.3, 27.8, 180]],
    'TASK_BLU_B_TFFT': [['ELEVATE'],
                        ['MOVE', -94.5, -47.4, 180],
                        ['MOVE', -157, -47.4, 180],
                        ['BALANCE']],
    'TASK_BLU_C_TFFT': [['ELEVATE'],
                        ['MOVE', -92.4, -128.5, 180],
                        ['MOVE', -188.1, -128.5, 180]],
    'TASK_RED_A_TFFT': [['ELEVATE'],
                        ['MOVE', 122.6, 27.8, 0],
                        ['MOVE', 238.3, 27.8, 0]],
    'TASK_RED_B_TFFT': [['ELEVATE'],
                        ['MOVE', 94.5, -47.4, 0],
                        ['MOVE', 157, -47.4, 0],
                        ['BALANCE']],
    'TASK_RED_C_TFFT': [['ELEVATE'],
                        ['MOVE', 92.4, -128.5, 0],
                        ['MOVE', 188.1, -128.5, 0]],
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
    'GRABBER': grabberConfig,
    'INTAKE': intakeConfig,
    'AUTON': autonConfig,
}

showbot = {
    'CONTROLLERS': controllerConfig,
    'SWERVOMETER': swervometerConfig, # Must be BEFORE drivetrain
    'VISION': visionConfig, # Must be BEFORE drivetrain
    'DRIVETRAIN': drivetrainConfig,
    'AUTON': autonConfig,
}

#showbot['DRIVETRAIN']['FRONTLEFT_DRIVEMOTOR'] = 1 # how to override just one thing

##########################
###  CONFIG TO DEPLOY  ###
##########################
robotconfig = showbot
