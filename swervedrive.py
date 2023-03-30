from navx import AHRS
import math
from util import clamp

#from magicbot import magiccomponent
import swervemodule
from dashboard import Dashboard

#import ntcore
from networktables import NetworkTables
from networktables.util import ntproperty
from collections import namedtuple
from wpimath.controller import PIDController
from swervometer import Swervometer
from logger import Logger
from robotconfig import MODULE_NAMES

DASH_PREFIX = MODULE_NAMES.SWERVEDRIVE

BalanceConfig = namedtuple('BalanceConfig', ['sd_prefix', 'balance_pitch_kP', 'balance_pitch_kI', 'balance_pitch_kD', 'balance_yaw_kP', 'balance_yaw_kI', 'balance_yaw_kD'])
TargetConfig = namedtuple('TargetConfig', ['sd_prefix', 'target_kP', 'target_kI', 'target_kD'])
BearingConfig = namedtuple('BearingConfig', ['sd_prefix', 'bearing_kP', 'bearing_kI', 'bearing_kD'])
VisionDriveConfig = namedtuple('VisionDriveConfig', ['sd_prefix', 'x_visionDrive_kP', 'x_visionDrive_kI', 'x_visionDrive_kD', 'y_visionDrive_kP', 'y_visionDrive_kI', 'y_visionDrive_kD', 'target_offsetX_reflective', 'target_target_size_reflective', 'target_offsetX_april', 'target_target_size_april', 'max_target_offset_x', 'min_target_size'])

class SwerveDrive:

    # Get some config options from the dashboard.
    # I'm pretty sure these don't get written anywhere else, unless via Shuffleboard
    # These are static class variables but they're accessed using self. later -- should be fixed
    # Also, it's weird to use ntproperty here when we do otherwise elsewhere
    lower_input_thresh = ntproperty('/SmartDashboard/drive/drive/lower_input_thresh', 0.001)
    rotation_multiplier = ntproperty('/SmartDashboard/drive/drive/rotation_multiplier', 0.5)
    xy_multiplier = ntproperty('/SmartDashboard/drive/drive/xy_multiplier', 0.65)
    debugging = ntproperty('/SmartDashboard/drive/drive/debugging', True) # Turn to true to run it in verbose mode.

    def __init__(
            self, 
            _frontLeftModule, 
            _frontRightModule, 
            _rearLeftModule, 
            _rearRightModule, 
            _swervometer, 
            _vision, 
            _gyro, 
            _balance_cfg, 
            _target_cfg, 
            _bearing_cfg,
            _visionDrive_cfg,
            _auton_steer_straight,
            _teleop_steer_straight):
        
        self.logger = Logger.getLogger()
        self.frontLeftModule = _frontLeftModule
        self.frontRightModule = _frontRightModule
        self.rearLeftModule = _rearLeftModule
        self.rearRightModule = _rearRightModule

        # Put all the modules into a dictionary
        self.modules = {
            'front_left': self.frontLeftModule,
            'front_right': self.frontRightModule,
            'rear_left': self.rearLeftModule,
            'rear_right': self.rearRightModule
        }

        self.swervometer = _swervometer
        self.vision = _vision
        self.gyro = _gyro
        self.gyro_angle_zero = 0.0
        #assuming balanced at initialization
        #self.gyro_balance_zero = self.getGyroRoll()
        self.gyro_balance_zero = 0.0

        # Get Smart Dashboard
        #self.sd = NetworkTables.getTable('SmartDashboard')
        self.dashboard = Dashboard.getDashboard()

        # should do this here rather than above
        # self.lower_input_thresh = ntproperty('/SmartDashboard/drive/drive/lower_input_thresh', 0.001)
        # self.rotation_multiplier = ntproperty('/SmartDashboard/drive/drive/rotation_multiplier', 0.5)
        # self.xy_multiplier = ntproperty('/SmartDashboard/drive/drive/xy_multiplier', 0.65)
        # self.debugging = ntproperty('/SmartDashboard/drive/drive/debugging', True) # Turn to true to run it in verbose mode.

        # Set all inputs to zero
        self._requested_vectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self._requested_angles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self._requested_speeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self.pose_target_x = 666 # impossible values, default for logging
        self.pose_target_y = 666 
        self.pose_target_bearing = 666 
        
        # Variables that allow enabling and disabling of features in code
        self.squared_inputs = False
        self.threshold_input_vectors = True

        #self.width = (30 / 12) / 2 # (Inch / 12 = Foot) / 2
        #self.length = (30 / 12) / 2 # (Inch / 12 = Foot) / 2

        self.wheel_lock = False
        
        self.balance_config = _balance_cfg
        self.balance_pitch_kP = self.balance_config.balance_pitch_kP
        self.balance_pitch_kI = self.balance_config.balance_pitch_kI
        self.balance_pitch_kD = self.balance_config.balance_pitch_kD

        self.balance_yaw_kP = self.balance_config.balance_yaw_kP
        self.balance_yaw_kI = self.balance_config.balance_yaw_kI
        self.balance_yaw_kD = self.balance_config.balance_yaw_kD

        self.balance_pitch_pid_controller = PIDController(self.balance_config.balance_pitch_kP, self.balance_config.balance_pitch_kI, self.balance_config.balance_pitch_kD)
        self.balance_pitch_pid_controller.enableContinuousInput(-180, 180)
        self.balance_pitch_pid_controller.setTolerance(0.5, 0.5) # may need to tweak this with PID testing

        self.balance_yaw_pid_controller = PIDController(self.balance_config.balance_yaw_kP, self.balance_config.balance_yaw_kI, self.balance_config.balance_yaw_kD)
        self.balance_yaw_pid_controller.enableContinuousInput(0, 360)
        self.balance_yaw_pid_controller.setTolerance(0.5, 0.5) # may need to tweak this with PID testing

        self.target_config = _target_cfg
        self.target_kP = self.target_config.target_kP
        self.target_kI = self.target_config.target_kI
        self.target_kD = self.target_config.target_kD
        
        self.target_x_pid_controller = PIDController(self.target_config.target_kP, self.target_config.target_kI, self.target_config.target_kD)
        self.target_x_pid_controller.setTolerance(5, 1)
        self.target_y_pid_controller = PIDController(self.target_config.target_kP, self.target_config.target_kI, self.target_config.target_kD)
        self.target_y_pid_controller.setTolerance(5, 1)
        # self.target_rcw_pid_controller = PIDController(self.target_config.target_kP, self.target_config.target_kI, self.target_config.target_kD)
        # self.target_rcw_pid_controller.setTolerance(0.5, 0.5)
        # self.target_rcw_pid_controller.enableContinuousInput(0, 360)

        self.bearing_config = _bearing_cfg
        self.bearing_kP = self.bearing_config.bearing_kP
        self.bearing_kI = self.bearing_config.bearing_kI
        self.bearing_kD = self.bearing_config.bearing_kD
        self.bearing_pid_controller = PIDController(self.bearing_kP, self.bearing_kI, self.bearing_kD)
        self.bearing_pid_controller.setTolerance(1, 1)
        
        self.bearing = self.getGyroAngle()
        self.updateBearing = False

        # TODO: 
        # - tune PID values
        self.visionDrive_config = _visionDrive_cfg
        self.visionDrive_x_pid_controller = PIDController(self.visionDrive_config.x_visionDrive_kP, self.visionDrive_config.x_visionDrive_kP, self.visionDrive_config.x_visionDrive_kP)
        self.visionDrive_x_pid_controller.setTolerance(0.5, 0.5)
        self.visionDrive_y_pid_controller = PIDController(self.visionDrive_config.y_visionDrive_kP, self.visionDrive_config.y_visionDrive_kP, self.visionDrive_config.y_visionDrive_kP)
        self.visionDrive_y_pid_controller.setTolerance(0.001, 0.001)
        self.reflectiveTargetOffsetX = self.visionDrive_config.target_offsetX_reflective
        self.reflectiveTargetTargetSize = self.visionDrive_config.target_target_size_reflective
        self.aprilTargetOffsetX = self.visionDrive_config.target_offsetX_april
        self.aprilTargetTargetSize = self.visionDrive_config.target_target_size_april
        self.max_target_offset_x = self.visionDrive_config.max_target_offset_x
        self.min_target_size = self.visionDrive_config.min_target_size

        self.inAuton = True
        self.autonSteerStraight = _auton_steer_straight
        self.teleopSteerStraight = _teleop_steer_straight

    def setInAuton(self, state):
        self.inAuton = state
        return

    def shouldSteerStraight(self):
        if self.inAuton:
            return self.autonSteerStraight
        else:
            return self.teleopSteerStraight

    def getBearing(self):      
        return self.bearing

    def setBearing(self, _bearing):      
        self.bearing = _bearing
        self.updateBearing = False

    def reset(self):
        self.log("SWERVETRIVE reset")

        # Set all inputs to zero
        self._requested_vectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self._requested_angles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self._requested_speeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }
        # Variables that allow enabling and disabling of features in code
        self.squared_inputs = False
        self.threshold_input_vectors = True

        self.wheel_lock = False
        
        for key in self.modules:
            self.modules[key].reset()

        self.resetGyro()
        self.bearing = self.getGyroAngle()
        self.updateBearing = False

    @staticmethod
    def square_input(input):
        return math.copysign(input * input, input) # Return magnitude of x but the sign of y. (x, y)
    
    @staticmethod
    def normalize(data):
        """
        Get the maximum value in the data. If the max is more than 1,
        divide each data by that max.
        :param data: The data to be normalized
        :returns: The normalized data
        """
        maxMagnitude = max(abs(x) for x in data)

        if maxMagnitude > 1.0:
            for i in range(len(data)):
                data[i] = data[i] / maxMagnitude
        
        return data

    @staticmethod
    def normalizeDictionary(data):
        """
        Get the maximum value in the data. If the max is more than 1,
        divide each data by that max.
        :param data: The dictionary with the data to be normalized
        :returns: The normalized dictionary with the data
        """
        maxMagnitude = max(abs(x) for x in data.values())

        if maxMagnitude > 1.0:
            for key in data:
                data[key] = data[key] / maxMagnitude
        
        return data

    #angle off of gyro zero
    def getGyroAngle(self):
        angle = (self.gyro.getAngle() - self.gyro_angle_zero + self.swervometer.getTeamGyroAdjustment()) % 360
        #print ("Gyro Adjustment", self.swervometer.getTeamGyroAdjustment())
        return angle
        
    def getGyroBalance(self):
        balance = (self.gyro.getPitch() - self.gyro_balance_zero)

        return balance

    #raw level side to side
    def getGyroPitch(self):
        pitch = self.gyro.getPitch()

        return pitch

    #raw angle
    def getGyroYaw(self):
        yaw = self.gyro.getYaw()

        return yaw

    #raw level front to back
    def getGyroRoll(self):
        roll = self.gyro.getRoll()

        return roll

    def printGyro(self):
        self.log("Angle: ", self.getGyroAngle(), ", Pitch: ", self.getGyroPitch(), ", Yaw: ", self.getGyroYaw(), ", Roll: ", self.getGyroRoll())

    def resetGyro(self):
        self.log("SWERVEDRIVE resetGyro Angle: ", self.getGyroAngle(), ", Pitch: ", self.getGyroPitch(), ", Yaw: ", self.getGyroYaw(), ", Roll: ", self.getGyroRoll())
        if self.gyro:
            self.gyro.reset()
            self.bearing = self.getGyroAngle()

    def flush(self):
        """
        This method should be called to reset all requested values of the drive system.
        It will also flush each module individually.
        """
        self._requested_vectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self._requested_angles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self._requested_speeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        for module in self.modules.values():
            module.flush()

    def set_raw_fwd(self, fwd):
        """
        Sets the raw fwd value to prevent it from being passed through any filters
        :param fwd: A value from -1 to 1
        """
        self._requested_vectors['fwd'] = fwd

    def set_raw_strafe(self, strafe):
        """
        Sets the raw strafe value to prevent it from being passed through any filters
        :param strafe: A value from -1 to 1
        """
        self._requested_vectors['strafe'] = strafe
    
    def set_raw_rcw(self, rcw):
        """
        Sets the raw rcw value to prevent it from being passed through any filters
        :param rcw: A value from -1 to 1
        """
        self._requested_vectors['rcw'] = rcw

    def set_fwd(self, fwd):
        """
        Individually sets the fwd value. (passed through filters)
        :param fwd: A value from -1 to 1
        """
        if self.squared_inputs:
            fwd = self.square_input(fwd)

        fwd *= self.xy_multiplier

        self._requested_vectors['fwd'] = fwd

    def set_strafe(self, strafe):
        """
        Individually sets the strafe value. (passed through filters)
        :param strafe: A value from -1 to 1
        """
        if self.squared_inputs:
            strafe = self.square_input(strafe)

        strafe *= self.xy_multiplier

        self._requested_vectors['strafe'] = strafe

    def set_rcw(self, rcw):
        """
        Individually sets the rcw value. (passed through filters)
        :param rcw: A value from -1 to 1
        """
        if self.squared_inputs:
            rcw = self.square_input(rcw)

        rcw *= self.rotation_multiplier

        self._requested_vectors['rcw'] = rcw

    def balance(self):
        
        self.log("Balance starting")

        #self.printGyro()

        yawSign = -1

        if(self.getGyroYaw() >= -90 and self.getGyroYaw() <= 90):
            BALANCED_YAW = 0.0
            yawSign = +1
        else:
            BALANCED_YAW = 180.0
            yawSign = -1
        BALANCED_PITCH = 0.0

        self.log("Balance: Yaw = ", self.getGyroYaw(), " BALANCED_YAW = ", BALANCED_YAW, " BALANCED_PITCH = ", BALANCED_PITCH)
        self.log("Balance: pitch:", self.getGyroBalance())
        pitch_error = self.balance_pitch_pid_controller.calculate(self.getGyroBalance(), BALANCED_PITCH)
        yaw_error = self.balance_yaw_pid_controller.calculate(self.getGyroYaw(), BALANCED_YAW)
        self.log("Balance: pitch_error:", pitch_error, ", yaw_error: ", yaw_error)

        # Set the output to 0 if at setpoint or to a value between (-1, 1)
        if self.balance_pitch_pid_controller.atSetpoint():
            pitch_output = 0
        else:
            #pitch_output = clamp(pitch_error)
            pitch_output = -pitch_error # Deliberately flipping sign
            pitch_output *= self.swervometer.getTeamMoveAdjustment()
        
        if self.balance_yaw_pid_controller.atSetpoint():
            yaw_output = 0
        else:
            yaw_output = clamp(yaw_error)
        
        self.log("Balance: Pitch setpoint: ", self.balance_pitch_pid_controller.getSetpoint(), "pitch output: ", pitch_output, " pitch error: ", pitch_error)
        self.log("Balance: Yaw setpoint: ", self.balance_yaw_pid_controller.getSetpoint(), "yaw output: ", yaw_output, " yaw error: ", yaw_error)

        # Put the output to the dashboard
        self.log('Balance pitch output', pitch_output)
        self.log('Balance yaw output', yaw_output)
        self.move(yawSign * pitch_output, 0.0, yaw_output, self.bearing)
        
        self.update_smartdash()

        self.execute('center')

        if self.balance_pitch_pid_controller.atSetpoint() and self.balance_yaw_pid_controller.atSetpoint():
            self.log("Balance: atSetpoint")
            return True
        else:
            self.log("Balance: not atSetpoint")
            return False

    def steerStraight(self, rcw, bearing):
        
        self.bearing = bearing
        current_angle = self.getGyroAngle()
        if rcw != 0:
            self.updateBearing = True
            self.log("rcw (!=0): ", rcw, " bearing: ", self.bearing, " currentAngle: ", current_angle)
            return rcw
        else:
            self.updateBearing = False
            angle_diff = abs(current_angle - self.bearing)
            if (angle_diff) > 180:
                angle_diff = 360 - angle_diff
                if self.bearing < current_angle:
                    target_angle = current_angle + angle_diff
                else:
                    target_angle = current_angle - angle_diff
            else:
                if self.bearing < current_angle:
                    target_angle = current_angle - angle_diff
                else:
                    target_angle = current_angle + angle_diff

            rcw_error = self.bearing_pid_controller.calculate(self.getGyroAngle(), target_angle)
            self.log("SWERVEDRIVE steerStraight rcw: ", rcw, " rcw_error: ", rcw_error, " current_angle: ", current_angle, " bearing: ", self.bearing, " target_angle: ", target_angle)
            return rcw_error

    def move(self, base_fwd, base_strafe, rcw, bearing):
        
        """
        Calulates the speed and angle for each wheel given the requested movement
        Positive fwd value = Forward robot movement\n
        Negative fwd value = Backward robot movement\n
        Positive strafe value = Left robot movement\n
        Negative strafe value = Right robot movement
        :param fwd: the requested movement in the X direction 2D plane
        :param strafe: the requested movement in the Y direction of the 2D plane
        :param rcw: the requestest magnitude of the rotational vector of a 2D plane
        """
        self.log("SWERVEDRIVE: MoveAdjustment: ", self.swervometer.getTeamMoveAdjustment())
        fwd = base_fwd #* self.swervometer.getTeamMoveAdjustment()
        strafe = base_strafe #* self.swervometer.getTeamMoveAdjustment()

        self.log("SWERVEDRIVE Moving:", fwd, strafe, rcw, bearing)

        #Convert field-oriented translate to chassis-oriented translate
        
        current_angle = self.getGyroAngle() % 360
        desired_angle = (math.degrees(math.atan2(strafe, fwd))) % 360
        chassis_angle = (desired_angle - current_angle) % 360
        magnitude = clamp(math.hypot(strafe, fwd), 0, 1)
        
        chassis_fwd = magnitude * math.sin(math.radians(chassis_angle))
        chassis_strafe = magnitude * math.cos(math.radians(chassis_angle))

        #self.log("modified strafe: " + str(chassis_strafe) + ", modified fwd: " + str(chassis_fwd))
        # self.dashboard.putNumber("Current Gyro Angle", self.getGyroAngle())

        self.set_fwd(chassis_fwd)
        self.set_strafe(chassis_strafe)

        # self.set_fwd(fwd)
        # self.set_strafe(strafe)
        
        self.log("Drivetrain: Move: shouldSteerStraight:", self.shouldSteerStraight())

        if self.shouldSteerStraight():
            self.set_rcw(self.steerStraight(rcw, bearing))
        else:
            self.set_rcw(rcw)

    def goToOffsetAndTargetSize(self, targetOffsetX, targetTargetSize):
        if self.vision:
            
            self.log("goToOffsetAndTargetSize: targetOffsetX: ", targetOffsetX, " targetTargetSize: ", targetTargetSize)
            YAW = 0.0
            if(self.getGyroYaw() >= -90 and self.getGyroYaw() <= 90):
                YAW = 0.0
            else:
                YAW = 180.0

            self.log("goToOffsetAndTargetSize: YAW: ", YAW)

            offsetX = self.vision.getTargetOffsetHorizontalReflective() 
            targetSize = self.vision.getTargetSizeReflective()

            self.log("goToOffsetAndTargetSize: offsetX: ", offsetX, " targetSize: ", targetSize)

            if abs(offsetX) > self.max_target_offset_x or targetSize < self.min_target_size: # impossible values, there's no target
                self.log('Aborting goToReflectiveTapeCentered() cuz no targets')
                self.log('Target offset X: ', abs(offsetX), ", Target area: ", targetSize)
                self.idle()
                return False

            x_error = self.visionDrive_x_pid_controller.calculate(offsetX, targetOffsetX)
            x_error = -x_error
            x_error = self.vision_drive_clamp(x_error, 0, 0.1)
            
            y_error = 10 * self.visionDrive_y_pid_controller.calculate(targetSize, targetTargetSize)
            #y_error = -y_error
            y_error = self.vision_drive_clamp(y_error, 0, 0.1)

            self.log("goToOffsetAndTargetSize: x_error: ", x_error, " y_error: ", y_error)
            
            #x_error = 0
            #y_error = 0

            if self.visionDrive_x_pid_controller.atSetpoint():
                if self.visionDrive_y_pid_controller.atSetpoint():
                    self.idle()
                    self.update_smartdash()
                    return True
                else:
                    self.move(0, y_error, 0, self.bearing)
                    self.execute('center')
                    self.update_smartdash()
                    return False
            else:
                #self.move(x_error, y_error, 0, YAW)
                self.move(x_error, 0, 0, self.bearing)
                self.execute('center')
                self.update_smartdash()
                return False

    def vision_drive_clamp(self, num, min_value, max_value):
        if num >= 0:
            return max(min(num, max_value), min_value)
        else:
            neg_min = -min_value
            neg_max = -max_value
            return max(min(num, neg_min), neg_max)

    def goToAprilTagCentered(self):
        self.vision.setToAprilTagPipeline()
        return self.goToOffsetAndTargetSize(self.aprilTagTargetOffsetX,
                                            self.aprilTagTargetTargetSize)

    def goToReflectiveTapeCentered(self):
        self.vision.setToReflectivePipeline()
        return self.goToOffsetAndTargetSize(self.reflectiveTargetOffsetX,
                                            self.reflectiveTargetTargetSize)

    def halfMoonBalance(self, checkpointX, checkpointY, cornerX, cornerY, bearing, tolerance):

        currentX, currentY, currentBearing = self.swervometer.getCOF()

        # Figure out if the bot needs to rotate right or left and turning on which corner.
        if checkpointY < cornerY:
            bearingAdjustment = 175 # Slightly less than 180 to indicate direction to turn.
            corner = 'rear_left'
            slideY = cornerY + 20
        else:
            bearingAdjustment = -175 # Slightly less than 180 to indicate direction to turn.
            corner = 'rear_right'
            slideY = cornerY - 20

        # Figure out where the bot needs to come back towards until it needs to balance.
        if checkpointX > 0:
            targetX = checkpointX + 50
        else:
            targetX = checkpointX - 50

        # Figure out the new bearing after a "180" (175).
        newBearing = (bearing + bearingAdjustment) % 360

        # Figure out if the bot is close to the right bearing.
        bearingDifference = abs(self.getBearing() - newBearing)

        # Final Stage: Balancing
        if abs(self.getGyroBalance()) > tolerance:
            self.log("Swervedrive: Half-Moon: Balancing")
            return self.balance()

        # Third Stage: If nearly rotated (don't wait for PID), move onto charge station:
        elif bearingDifference < 10:
            self.goToPose(targetX, currentY, newBearing)
            self.execute('center')
            self.log("Swervedrive: Half-Moon: Moving back")
            return False
            
        # Second Stage: Start corner pivot (don't wait for PID) with a slight Y shift
        elif abs(currentX) <= abs(cornerX):
            self.goToPose(currentX, slideY, newBearing)
            self.execute(corner)
            self.log("Swervedrive: Half-Moon: Rotating")
            return False
        
        # First Stage: Move to a point just past the pivot corner.
        else:
            self.goToPose(checkpointX, checkpointY, bearing)
            self.log("Swervedrive: Half-Moon: Moving")
            return False
       
    def goToBalance(self, x, y, bearing, tolerance):
        self.log("SWERVEDRIVE Going to balance:", x, y, bearing, tolerance)

        if abs(self.getGyroBalance()) > tolerance:
            return True
        else:
            return self.goToPose(x, y, bearing)

    def goToPose(self, x, y, bearing):

        self.log("SWERVEDRIVE Going to pose:", x, y, bearing)

        # for telemetry
        self.pose_target_x = x
        self.pose_target_y = y
        self.pose_target_bearing = bearing

        currentX, currentY, currentBearing = self.swervometer.getCOF()
        x_error = self.target_x_pid_controller.calculate(currentX, x)
        y_error = -self.target_y_pid_controller.calculate(currentY, y)
        
        #self.log("hello: x: ", self.target_x_pid_controller.getSetpoint(), " y: ", self.target_y_pid_controller.getSetpoint())
        if self.target_x_pid_controller.atSetpoint():
            self.log("X at set point")
        if self.target_y_pid_controller.atSetpoint():
            self.log("Y at set point")
        
        # Get current pose                    
        currentX, currentY, currentBearing = self.swervometer.getCOF()
        
        if self.target_x_pid_controller.atSetpoint() and self.target_y_pid_controller.atSetpoint(): 
            self.update_smartdash()
            return True
        else:
            self.move(x_error, y_error, 0, bearing)
            
            self.update_smartdash()
            self.execute('center')
            # self.log("xPositionError: ", self.target_x_pid_controller.getPositionError(), "yPositionError: ", self.target_y_pid_controller.getPositionError(), "rcwPositionError: ", self.target_rcw_pid_controller.getPositionError())
            # self.log("xPositionTolerance: ", self.target_x_pid_controller.getPositionTolerance(), "yPositionTolerance: ", self.target_y_pid_controller.getPositionTolerance(), "rcwPositionTolerance: ", self.target_rcw_pid_controller.getPositionTolerance())
            # self.log("currentX: ", currentX, " targetX: ", x, "x_error: ", x_error, " currentY: ", currentY, " targetY: ", y, " y_error: ", y_error, " currentBearing: ", currentRCW, " self.bearing: ", self.bearing, " target bearing: ", bearing)
            return False

    def _calculate_vectors(self):
        """
        Calculate the requested speed and angle of each modules from self._requested_vectors and store them in
        self._requested_speeds and self._requested_angles dictionaries.
        """
        self._requested_vectors['fwd'], self._requested_vectors['strafe'], self._requested_vectors['rcw'] = self.normalize([self._requested_vectors['fwd'], self._requested_vectors['strafe'], self._requested_vectors['rcw']])

        # Does nothing if the values are lower than the input thresh
        if self.threshold_input_vectors:
            #self.log("checking thresholds: fwd: ", self._requested_vectors['fwd'], "strafe: ", self._requested_vectors['strafe'], "rcw: ", self._requested_vectors['rcw'])
            if abs(self._requested_vectors['fwd']) < self.lower_input_thresh:
                #self.log("forward = 0")
                self._requested_vectors['fwd'] = 0

            if abs(self._requested_vectors['strafe']) < self.lower_input_thresh:
                #self.log("strafe = 0")
                self._requested_vectors['strafe'] = 0

            if abs(self._requested_vectors['rcw']) < self.lower_input_thresh:
                #self.log("rcw = 0")
                self._requested_vectors['rcw'] = 0

            if self._requested_vectors['rcw'] == 0 and self._requested_vectors['strafe'] == 0 and self._requested_vectors['fwd'] == 0:  # Prevents a useless loop.
                #self.log("all three zero")
                self._requested_speeds = dict.fromkeys(self._requested_speeds, 0) # Do NOT reset the wheel angles.

                if self.wheel_lock:
                    # This is intended to set the wheels in such a way that it
                    # difficult to push the robot (intended for defense)

                    self._requested_angles['front_left'] = -45
                    self._requested_angles['front_right'] = 45
                    self._requested_angles['rear_left'] = 45
                    self._requested_angles['rear_right'] = -45

                    #self.wheel_lock = False
                    #self.log("testing wheel lock")
                return
        
        frame_dimension_x, frame_dimension_y = self.swervometer.getFrameDimensions()
        ratio = math.hypot(frame_dimension_x, frame_dimension_y)

        rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
        leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
        rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
        frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
        
        # Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
        rearLeft_speed = math.hypot(frontX, rightY)
        rearLeft_angle = math.degrees(math.atan2(frontX, rightY))

        frontLeft_speed = math.hypot(frontX, leftY)
        frontLeft_angle = math.degrees(math.atan2(frontX, leftY))

        rearRight_speed = math.hypot(rearX, rightY)
        rearRight_angle = math.degrees(math.atan2(rearX, rightY))

        frontRight_speed = math.hypot(rearX, leftY)
        frontRight_angle = math.degrees(math.atan2(rearX, leftY))

        self._requested_speeds['front_left'] = frontLeft_speed
        self._requested_speeds['front_right'] = frontRight_speed
        self._requested_speeds['rear_left'] = rearLeft_speed
        self._requested_speeds['rear_right'] = rearRight_speed

        self._requested_angles['front_left'] = frontLeft_angle
        self._requested_angles['front_right'] = frontRight_angle
        self._requested_angles['rear_left'] = rearLeft_angle
        self._requested_angles['rear_right'] = rearRight_angle

        self._requested_speeds = self.normalizeDictionary(self._requested_speeds)

        # Zero request vectors for saftey reasons
        self._requested_vectors['fwd'] = 0.0
        self._requested_vectors['strafe'] = 0.0
        self._requested_vectors['rcw'] = 0.0

    def _calculate_swoop_vectors(self, axis_of_rotation):
        """
        Calculate the requested speed and angle of each modules from self._requested_vectors and store them in
        self._requested_speeds and self._requested_angles dictionaries.
        """
        self._requested_vectors['fwd'], self._requested_vectors['strafe'], self._requested_vectors['rcw'] = self.normalize([self._requested_vectors['fwd'], self._requested_vectors['strafe'], self._requested_vectors['rcw']])

        # Does nothing if the values are lower than the input thresh
        if self.threshold_input_vectors:
            #self.log("checking thresholds: fwd: ", self._requested_vectors['fwd'], "strafe: ", self._requested_vectors['strafe'], "rcw: ", self._requested_vectors['rcw'])
            if abs(self._requested_vectors['fwd']) < self.lower_input_thresh:
                #self.log("forward = 0")
                self._requested_vectors['fwd'] = 0

            if abs(self._requested_vectors['strafe']) < self.lower_input_thresh:
                #self.log("strafe = 0")
                self._requested_vectors['strafe'] = 0

            if abs(self._requested_vectors['rcw']) < self.lower_input_thresh:
                #self.log("rcw = 0")
                self._requested_vectors['rcw'] = 0

            if self._requested_vectors['rcw'] == 0 and self._requested_vectors['strafe'] == 0 and self._requested_vectors['fwd'] == 0:  # Prevents a useless loop.
                #self.log("all three zero")
                self._requested_speeds = dict.fromkeys(self._requested_speeds, 0) # Do NOT reset the wheel angles.

                if self.wheel_lock:
                    # This is intended to set the wheels in such a way that it
                    # difficult to push the robot (intended for defense)

                    self._requested_angles['front_left'] = 45
                    self._requested_angles['front_right'] = -45
                    self._requested_angles['rear_left'] = -45
                    self._requested_angles['rear_right'] = 45

                    #self.wheel_lock = False
                    #self.log("testing wheel lock"),
                return
        
        frame_dimension_x, frame_dimension_y = self.swervometer.getFrameDimensions()
        
        #frame_dimension_x *= 2 # Frame is effectively twice as big.
        #frame_dimension_y *= 2 # Frame is effectively twice as big.

        ratio = math.hypot(frame_dimension_x, frame_dimension_y)

        self.log("Swoop: fwd: ", self._requested_vectors['fwd'], " strafe: ", self._requested_vectors['strafe'], "rcw: ", self._requested_vectors['rcw'])

        if (axis_of_rotation == 'front_left'):
            #rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            #leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            #rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
            #frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
        
            # Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
            frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * 0)
            rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * 1)
            rearLeft_speed = math.hypot(frontX, rightY)
            rearLeft_angle = math.degrees(math.atan2(frontX, rightY))

            frontX = self._requested_vectors['strafe']
            leftY = self._requested_vectors['fwd']
            frontLeft_speed = math.hypot(frontX, leftY)
            frontLeft_angle = math.degrees(math.atan2(frontX, leftY))

            rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
            rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            rearRight_speed = math.hypot(rearX, rightY)
            rearRight_angle = math.degrees(math.atan2(rearX, rightY))

            rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * 1)
            leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * 0)
            frontRight_speed = math.hypot(rearX, leftY)
            frontRight_angle = math.degrees(math.atan2(rearX, leftY))
        elif (axis_of_rotation == 'front_right'):
            #rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            #leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            #rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
            #frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
        
            # Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
            frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
            rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            rearLeft_speed = math.hypot(frontX, rightY)
            rearLeft_angle = math.degrees(math.atan2(frontX, rightY))

            frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * 1)
            leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * 0)
            frontLeft_speed = math.hypot(frontX, leftY)
            frontLeft_angle = math.degrees(math.atan2(frontX, leftY))

            rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * 0)
            rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * 1)
            rearRight_speed = math.hypot(rearX, rightY)
            rearRight_angle = math.degrees(math.atan2(rearX, rightY))

            rearX = self._requested_vectors['strafe']
            leftY = self._requested_vectors['fwd']
            frontRight_speed = math.hypot(rearX, leftY)
            frontRight_angle = math.degrees(math.atan2(rearX, leftY))
        elif (axis_of_rotation == 'rear_right'):
            #rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            #leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            #rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
            #frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
        
            # Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
            frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * 1)
            rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * 0)
            rearLeft_speed = math.hypot(frontX, rightY)
            rearLeft_angle = math.degrees(math.atan2(frontX, rightY))

            frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
            leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            frontLeft_speed = math.hypot(frontX, leftY)
            frontLeft_angle = math.degrees(math.atan2(frontX, leftY))

            rearX = self._requested_vectors['strafe']
            rightY = self._requested_vectors['fwd']
            rearRight_speed = math.hypot(rearX, rightY)
            rearRight_angle = math.degrees(math.atan2(rearX, rightY))

            rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * 0)
            leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * 1)
            frontRight_speed = math.hypot(rearX, leftY)
            frontRight_angle = math.degrees(math.atan2(rearX, leftY))
        elif (axis_of_rotation == 'rear_left'):
            #rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            #leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            #rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
            #frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
        
            # Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
            frontX = self._requested_vectors['strafe']
            rightY = self._requested_vectors['fwd']
            rearLeft_speed = math.hypot(frontX, rightY)
            rearLeft_angle = math.degrees(math.atan2(frontX, rightY))

            frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * 0)
            leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * 1)
            frontLeft_speed = math.hypot(frontX, leftY)
            frontLeft_angle = math.degrees(math.atan2(frontX, leftY))

            rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * 1)
            rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * 0)
            rearRight_speed = math.hypot(rearX, rightY)
            rearRight_angle = math.degrees(math.atan2(rearX, rightY))

            rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
            leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
            frontRight_speed = math.hypot(rearX, leftY)
            frontRight_angle = math.degrees(math.atan2(rearX, leftY))
        else:
            frontLeft_speed = 0
            frontRight_speed = 0
            rearLeft_speed = 0
            rearRight_speed = 0
            frontLeft_angle = 0
            frontRight_angle = 0
            rearLeft_angle = 0
            rearRight_angle = 0

        self._requested_speeds['front_left'] = frontLeft_speed
        self._requested_speeds['front_right'] = frontRight_speed
        self._requested_speeds['rear_left'] = rearLeft_speed
        self._requested_speeds['rear_right'] = rearRight_speed

        self._requested_angles['front_left'] = frontLeft_angle
        self._requested_angles['front_right'] = frontRight_angle
        self._requested_angles['rear_left'] = rearLeft_angle
        self._requested_angles['rear_right'] = rearRight_angle

        self._requested_speeds = self.normalizeDictionary(self._requested_speeds)

        # Zero request vectors for saftey reasons
        self._requested_vectors['fwd'] = 0.0
        self._requested_vectors['strafe'] = 0.0
        self._requested_vectors['rcw'] = 0.0

    def setWheelLock(self, isLocked):
        #self.log("is locked", isLocked)
        self.wheel_lock = isLocked
    
    def getWheelLock(self):
        return self.wheel_lock

    def setRampRates(self, openLoopRampRate, closedLoopRampRate):
        for key in self.modules:
            self.modules[key].setRampRate(openLoopRampRate, closedLoopRampRate)
        
    def debug(self, debug_modules=False):
        """
        Prints debugging information to log
        """
        if debug_modules:
            for key in self.modules:
                self.modules[key].debug()
        
        self.log('Requested values: ', self._requested_vectors, '\n')
        self.log('Requested angles: ', self._requested_angles, '\n')
        self.log('Requested speeds: ', self._requested_speeds, '\n')

    def execute(self, axis_of_rotation):
        """
        Sends the speeds and angles to each corresponding wheel module.
        Executes the doit in each wheel module.
        """
        self.update_smartdash()

        self.log("Swervedrive: Execute: axis_of_rotation: ", axis_of_rotation)

        if axis_of_rotation == 'center':
            self.log("Swervedrive: No swoop")
            self._calculate_vectors()
        else:
            self.log("Swervedrive: Swoop")
            self._calculate_swoop_vectors(axis_of_rotation)

        # Set the speed and angle for each module

        # Calculate normalized speeds with lever arm adjustment
        for key in self.modules:
            #self.log("Execute: key: ", key, " base speed: ", self._requested_speeds[key], " COMmult: ", self.swervometer.getCOMmult(key), " adjusted speed: ", (self._requested_speeds[key] * self.swervometer.getCOMmult(key)), self._requested_speeds[key] * self.swervometer.getCOMmult(key))
            self._requested_speeds[key] = self._requested_speeds[key] * self.swervometer.getCOMmult(key)
        
        self._requested_speeds = self.normalizeDictionary(self._requested_speeds)

        for key in self.modules:
            self.modules[key].move(self._requested_speeds[key], self._requested_angles[key])
        
        # Reset the speed back to zero
        self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)

        # Execute each module
        first_module = True
        for key in self.modules:
            self.log("Module: Key: ", key)
            self.modules[key].execute()

        COFX, COFY, COFAngle = self.swervometer.calculateCOFPose(self.modules, self.getGyroAngle())

        if self.vision:
            self.log("Vision started")
            if self.vision.canUpdatePose():
                self.log("Vision: canupdatepose")
                pose = self.vision.getPose()
                orientation = self.vision.getOrientation()
                if self.vision.shouldUpdatePose():
                    if pose[0] != -1:
                        self.swervometer.setCOF(pose[0], pose[1], orientation[2])
                        self.log("Vision updated position: (" + str(pose[0]) + ", " + str(pose[1]) + ") with rotation of " + str(orientation[2]) + " degrees.")
                    else:
                        self.log("Vision should have updated position, but pose was empty.")
                else:
                    self.log("Vision reports position: (" + str(pose[0]) + ", " + str(pose[1]) + ") with rotation of " + str(orientation[2]) + " degrees.")
                self.log("AFTER COMMENTS")

        self.log("COFX: ", COFX, ", COFY: ", COFY, ", COF Angle: ", COFAngle)

        if(self.updateBearing):
            self.log("Old Bearing: ", self.bearing)
            self.bearing = self.getGyroAngle()
            self.log("New Bearing: ", self.bearing)
            self.updateBearing = False

    def idle(self):
        for key in self.modules:
            self.modules[key].idle()
            
    def update_smartdash(self):
        """
        Log current state for telemetry
        """
        self.dashboard.putNumber(DASH_PREFIX, '/front_left_req_ang', self._requested_angles['front_left'])
        self.dashboard.putNumber(DASH_PREFIX, '/front_right_req_ang', self._requested_angles['front_right'])
        self.dashboard.putNumber(DASH_PREFIX, '/rear_left_req_ang', self._requested_angles['rear_left'])
        self.dashboard.putNumber(DASH_PREFIX, '/rear_right_req_ang', self._requested_angles['rear_right'])
        
        self.dashboard.putNumber(DASH_PREFIX, '/front_left_req_spd', self._requested_speeds['front_left'])
        self.dashboard.putNumber(DASH_PREFIX, '/front_right_req_ang', self._requested_speeds['front_right'])
        self.dashboard.putNumber(DASH_PREFIX, '/rear_left_req_ang', self._requested_speeds['rear_left'])
        self.dashboard.putNumber(DASH_PREFIX, '/rear_right_req_ang', self._requested_speeds['rear_right'])
        
        # Zero request vectors for saftey reasons
        self.dashboard.putNumber(DASH_PREFIX, '/req_vector_fwd', self._requested_vectors['fwd'])
        self.dashboard.putNumber(DASH_PREFIX, '/req_vector_strafe', self._requested_vectors['strafe'])
        self.dashboard.putNumber(DASH_PREFIX, '/req_vector_rcw', self._requested_vectors['rcw'])
        self.dashboard.putBoolean(DASH_PREFIX, '/wheel_lock', self.wheel_lock)
        
        self.dashboard.putNumber(DASH_PREFIX, '/pose_target_x', self.pose_target_x)
        self.dashboard.putNumber(DASH_PREFIX, '/pose_target_y', self.pose_target_y)
        self.dashboard.putNumber(DASH_PREFIX, '/pose_target_bearing', self.pose_target_bearing)
        
        self.dashboard.putNumber(DASH_PREFIX, '/Bearing', self.getBearing())
        self.dashboard.putNumber(DASH_PREFIX, '/Gyro Angle', self.getGyroAngle())
        self.dashboard.putNumber(DASH_PREFIX, '/Gyro Balance', self.getGyroBalance())
        self.dashboard.putNumber(DASH_PREFIX, '/Gyro Pitch', self.getGyroPitch())
        self.dashboard.putNumber(DASH_PREFIX, '/Bearing', self.getGyroRoll())
        self.dashboard.putNumber(DASH_PREFIX, '/Bearing', self.getGyroYaw())
        
        self.dashboard.putNumber(DASH_PREFIX, '/Balance Pitch kP', self.balance_pitch_pid_controller.getP())
        self.dashboard.putNumber(DASH_PREFIX, '/Balance Pitch kI', self.balance_pitch_pid_controller.getI())
        self.dashboard.putNumber(DASH_PREFIX, '/Balance Pitch kD', self.balance_pitch_pid_controller.getD())

        self.dashboard.putNumber(DASH_PREFIX, '/Balance Yaw kP', self.balance_yaw_pid_controller.getP())
        self.dashboard.putNumber(DASH_PREFIX, '/Balance Yaw kI', self.balance_yaw_pid_controller.getI())
        self.dashboard.putNumber(DASH_PREFIX, '/Balance Yaw kD', self.balance_yaw_pid_controller.getD())

        for key in self._requested_angles:
            self.dashboard.putNumber(DASH_PREFIX, '/%s_angle' % key, self._requested_angles[key])
            self.dashboard.putNumber(DASH_PREFIX, '/%s_speed' % key, self._requested_speeds[key])
                
    def log(self, *dataToLog):
        self.logger.log(DASH_PREFIX, dataToLog)   
