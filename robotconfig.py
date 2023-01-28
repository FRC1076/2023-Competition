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
    'HEADING_KP': 0.005,
    'HEADING_KI': 0.00001,
    'HEADING_KD':  0.00001,
    'BALANCE_PITCH_KP': 0.005,
    'BALANCE_PITCH_KI': 0.00001,
    'BALANCE_PITCH_KD':  0.00175,
    'BALANCE_YAW_KP': 0.005,
    'BALANCE_YAW_KI': 0.00001,
    'BALANCE_YAW_KD': 0.00001,
}

swervometerConfig = { # All positions measured in inches
    'TEAM_IS_RED': True, # Is the robot part of the Red Team?
    'FIELD_ORIGIN_X': 0.0, # X-Coordinate of field orgin (lower left from red start)
    'FIELD_ORIGIN_Y': 0.0, # Y-Coordinate of field orgin (lower left from red start)
    'FIELD_RED_A_START_POSITION_X': 100.0, # X-Coordinate of starting position A when on red team
    'FIELD_RED_A_START_POSITION_Y': 100.0, # Y-Coordinate of starting postion A when on red team
    'FIELD_RED_B_START_POSITION_X': 100.0, # X-Coordinate of starting position B when on red team
    'FIELD_RED_B_START_POSITION_Y': 400.0, # Y-Coordinate of starting postion B when on red team
    'FIELD_RED_C_START_POSITION_X': 100.0, # X-Coordinate of starting position C when on red team
    'FIELD_RED_C_START_POSITION_Y': 800.0, # Y-Coordinate of starting postion C when on red team
    'FIELD_BLU_A_START_POSITION_X': 100.0, # X-Coordinate of starting position A when on blue team
    'FIELD_BLU_A_START_POSITION_Y': 100.0, # Y-Coordinate of starting postion A when on blue team
    'FIELD_BLU_B_START_POSITION_X': 100.0, # X-Coordinate of starting position B when on blue team
    'FIELD_BLU_B_START_POSITION_Y': 400.0, # Y-Coordinate of starting postion B when on blue team
    'FIELD_BLU_C_START_POSITION_X': 100.0, # X-Coordinate of starting position C when on blue team
    'FIELD_BLU_C_START_POSITION_Y': 800.0, # Y-Coordinate of starting postion C when on blue team
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
    'ROBOT_FRAME_DIMENSION_X': 36.0, # X-coordinate length of robot frame
    'ROBOT_FRAME_DIMENSION_Y': 36.0, # Y-coordinate length of robot frame
    'ROBOT_BUMPER_DIMENSION_X': 4.0, # Width of bumper (X-axis)
    'ROBOT_BUMPER_DIMENSION_Y': 4.0, # Width of bumper (Y-axis)
    'ROBOT_GYRO_OFFSET_X': 10.0, # X-offset of center of gyro (relative to lower left frame)
    'ROBOT_GYRO_OFFSET_Y': 5.0, # Y-offset of center of gyro (relative to lower left frame)
    'ROBOT_CAMERA_OFFSET_X': 10.0, # X-offset of center of camera lens (relative to lower left frame)
    'ROBOT_CAMERA_OFFSET_Y': 20.0, # Y-offset of center of camera lens (relative to lower left frame)
}

shooterConfig = {
    'SHOOTER_ID': 10,
    'SHOOTER_RPM': 3500,
}

intakeConfig = {
    # Update IDs when known
    'INTAKE_MOTOR_ID': 17,
    'INTAKE_SOLENOID_FORWARD_ID': 1,
    'INTAKE_SOLENOID_REVERSE_ID': 2,
}

feederConfig = {
    'FEEDER_ID' : 9,
    'FEEDER_SPEED': 0.8,
}

tiltShooterConfig = {
    'TILTSHOOTER_ID': 1,
    'ROTATIONS_PER_360': 75,
    'MIN_DEGREES': 5,
    'MAX_DEGREES': 25,
    'BUFFER_DEGREES': 2,
    'SPEED': 0.1,
}

aimerConfig = {
    'AIMING_ROTATION_SPEED': 0.6,
    'AIMING_ACCURACY_DEGREES': 3,
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
    'HOOK_UP_TIME': 0.5,
    'DRIVE_FORWARD_TIME': 0.75,
    'HOOK_DOWN_TIME': 2.0,
    'DRIVE_BACKWARD_TIME': 2.5,
    'AUTON_SPEED_FORWARD': 0.8,
    'AUTON_SPEED_BACKWARD': 0.5,
}

climberConfig = {
    'WINCH_LEFT_ID': 6,
    'WINCH_RIGHT_ID': 14,
    # Pneumatic board IDs
    'SOLENOID_FORWARD_ID': 6,
    'SOLENOID_REVERSE_ID': 0,
    # DIO pin numbers
    'LEFT_LIMIT_ID': 0,
    'RIGHT_LIMIT_ID': 1,
    'CABLE_WRAPPED': 'UNDER',
    # Both speeds positive.
    # Extend speed must be lower than natural extend rate
    'EXTEND_SPEED': 0.2,
    'RETRACT_SPEED': 0.5,
}

hooksConfig = {
    'FRONT_HOOK_ID': 5,
    'BACK_HOOK_ID': 6,
    'LEFT_HOOK_ID': 7,
    'RIGHT_HOOK_ID': 8,

    'FRONT_TOP_PORT': 0,
    'FRONT_BOTTOM_PORT': 1,
    'BACK_TOP_PORT': 2,
    'BACK_BOTTOM_PORT': 3,
    'LEFT_TOP_PORT': 4,
    'LEFT_BOTTOM_PORT': 5,
    'RIGHT_TOP_PORT': 6,
    'RIGHT_BOTTOM_PORT': 7
}

#######################
###  ROBOT CONFIGS  ###
#######################
testbot = { # Always used for unit tests ($ python robot.py sim)
    'CONTROLLERS': controllerConfig,
    'DRIVETRAIN': drivetrainConfig,
    'SHOOTER': shooterConfig,
    'INTAKE': intakeConfig,
    'FEEDER': feederConfig,
    'TILTSHOOTER': tiltShooterConfig,
    'AIMER': aimerConfig,
    'VISION': visionConfig,
    'AUTON': autonConfig,
    'CLIMBER': climberConfig,
    'HOOKS': hooksConfig
}

gull_lake = {
    'CONTROLLERS': controllerConfig,
    'DRIVETRAIN': drivetrainConfig,
    'AIMER': aimerConfig,
    'VISION': visionConfig,
    'TILTSHOOTER': tiltShooterConfig,
    'SHOOTER': shooterConfig,
    'FEEDER': feederConfig,
    'AUTON': autonConfig,
    'CLIMBER': climberConfig,
    'HOOKS': hooksConfig
}

showbot = {
    'CONTROLLERS': controllerConfig,
    'DRIVETRAIN': drivetrainConfig,
    'AIMER': aimerConfig,
    'VISION': visionConfig,
    'SHOOTER': shooterConfig,
    'HOOKS': hooksConfig
}

testBot = {
    'CONTROLLERS': controllerConfig,
    'DRIVETRAIN': drivetrainConfig,
    'HOOKS': hooksConfig,
    'AUTON': autonConfig
}

showbot['SHOOTER']['SHOOTER_ID'] = 10 # how to override just one thing

##########################
###  CONFIG TO DEPLOY  ###
##########################
robotconfig = testBot