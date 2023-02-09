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
    'TEAM_IS_RED': True, # Is the robot part of the Red Team?
    'FIELD_START_POSITION': 'A', # Which of three starting positions is selected?
    'HAS_BUMPERS_ATTACHED': True, # Does the robot currently have bumpers attached?
    'FIELD_ORIGIN_X': 0.0, # X-Coordinate of field orgin (lower left from red start)
    'FIELD_ORIGIN_Y': 0.0, # Y-Coordinate of field orgin (lower left from red start)
    'FIELD_RED_A_START_POSITION_X': 159.0, # X-Coordinate of starting position A when on red team
    'FIELD_RED_A_START_POSITION_Y': 54.25, # Y-Coordinate of starting postion A when on red team
    'FIELD_RED_A_START_ANGLE': 0.0, # Heading angle of starting position A when on red team
    'FIELD_RED_B_START_POSITION_X': 225.0, # X-Coordinate of starting position B when on red team
    'FIELD_RED_B_START_POSITION_Y': 54.25, # Y-Coordinate of starting postion B when on red team
    'FIELD_RED_B_START_ANGLE': 0.0, # Heading angle of starting position B when on red team
    'FIELD_RED_C_START_POSITION_X': 291.0, # X-Coordinate of starting position C when on red team
    'FIELD_RED_C_START_POSITION_Y': 54.25, # Y-Coordinate of starting postion C when on red team
    'FIELD_RED_C_START_ANGLE': 0.0, # Heading angle of starting position C when on red team
    'FIELD_BLU_A_START_POSITION_X': 100.0, # X-Coordinate of starting position A when on blue team
    'FIELD_BLU_A_START_POSITION_Y': 100.0, # Y-Coordinate of starting postion A when on blue team
    'FIELD_BLU_A_START_ANGLE': 180.0, # Heading angle of starting position A when on blue team
    'FIELD_BLU_B_START_POSITION_X': 100.0, # X-Coordinate of starting position B when on blue team
    'FIELD_BLU_B_START_POSITION_Y': 400.0, # Y-Coordinate of starting postion B when on blue team
    'FIELD_BLU_B_START_ANGLE': 180.0, # Heading angle of starting position B when on blue team
    'FIELD_BLU_C_START_POSITION_X': 100.0, # X-Coordinate of starting position C when on blue team
    'FIELD_BLU_C_START_POSITION_Y': 800.0, # Y-Coordinate of starting postion C when on blue team
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
    'ROBOT_FRAME_DIMENSION_X': 26.0, # X-coordinate length of robot frame
    'ROBOT_FRAME_DIMENSION_Y': 34.0, # Y-coordinate length of robot frame
    'ROBOT_BUMPER_DIMENSION_X': 4.0, # Width of bumper (X-axis)
    'ROBOT_BUMPER_DIMENSION_Y': 4.0, # Width of bumper (Y-axis)
    'ROBOT_COM_OFFSET_X': 13.0, # X-offset of center of mass (assume half frame dimension)
    'ROBOT_COM_OFFSET_Y': 17.0, # Y-offset of center of mass (assume half frame dimension)
    'ROBOT_GYRO_OFFSET_X': 10.0, # X-offset of center of gyro (relative to lower left frame)
    'ROBOT_GYRO_OFFSET_Y': 5.0, # Y-offset of center of gyro (relative to lower left frame)
    'ROBOT_CAMERA_OFFSET_X': 10.0, # X-offset of center of camera lens (relative to lower left frame)
    'ROBOT_CAMERA_OFFSET_Y': 20.0, # Y-offset of center of camera lens (relative to lower left frame)
    'ROBOT_SWERVE_MODULE_OFFSET_X': 9.75, # X-offset of swerve module center from COM
    'ROBOT_SWERVE_MODULE_OFFSET_Y': 13.75, # X-offset of swerve module center from COM
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
    'ROBOT_INCHES_PER_ROTATION': 1.8035068937, # Inches per rotation of wheels
}

visionConfig = {
    'TARGET_HEIGHT': 8.5,
    'TARGET_RADIUS': 2,
    'SHOOTER_HEIGHT': 3.5,
    'SHOOTER_OFFSET': 1,
    'CAMERA_HEIGHT': 4,
    'CAMERA_PITCH': 0,
}

autonConfig = {
    'SCORE_EXISTING': True,
    'PICKUP_NEW': False,
    'SCORE_NEW': False,
    'BALANCE_BOT': True,
}

#######################
###  ROBOT CONFIGS  ###
#######################
testbot = { # Always used for unit tests ($ python robot.py sim)
    'CONTROLLERS': controllerConfig,
    'SWERVOMETER': swervometerConfig, # Must be BEFORE drivetrain
    'DRIVETRAIN': drivetrainConfig,
    'AUTON': autonConfig,
}

showbot = {
    'CONTROLLERS': controllerConfig,
    'SWERVOMETER': swervometerConfig, # Must be BEFORE drivetrain
    'DRIVETRAIN': drivetrainConfig,
    'AUTON': autonConfig,
}

#showbot['DRIVETRAIN']['FRONTLEFT_DRIVEMOTOR'] = 1 # how to override just one thing

##########################
###  CONFIG TO DEPLOY  ###
##########################
robotconfig = testbot