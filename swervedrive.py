from navx import AHRS
import math
from util import clamp

#from magicbot import magiccomponent
import swervemodule

#import ntcore
from networktables import NetworkTables
from networktables.util import ntproperty
from collections import namedtuple
from wpimath.controller import PIDController
from swervometer import Swervometer

BalanceConfig = namedtuple('BalanceConfig', ['sd_prefix', 'balance_pitch_kP', 'balance_pitch_kI', 'balance_pitch_kD', 'balance_yaw_kP', 'balance_yaw_kI', 'balance_yaw_kD'])
TargetConfig = namedtuple('TargetConfig', ['sd_prefix', 'target_kP', 'target_kI', 'target_kD'])
BearingConfig = namedtuple('BearingConfig', ['sd_prefix', 'bearing_kP', 'bearing_kI', 'bearing_kD'])

class SwerveDrive:

    # Get some config options from the dashboard.
    lower_input_thresh = ntproperty('/SmartDashboard/drive/drive/lower_input_thresh', 0.001)
    rotation_multiplier = ntproperty('/SmartDashboard/drive/drive/rotation_multiplier', 0.5)
    xy_multiplier = ntproperty('/SmartDashboard/drive/drive/xy_multiplier', 0.65)
    debugging = ntproperty('/SmartDashboard/drive/drive/debugging', True) # Turn to true to run it in verbose mode.

    def __init__(self, _frontLeftModule, _frontRightModule, _rearLeftModule, _rearRightModule, _swervometer, _vision, _gyro, _balance_cfg, _target_cfg, _bearing_cfg):
        
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
        self.sd = NetworkTables.getTable('SmartDashboard')

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

        self.sd.putNumber('Balance Pitch kP', self.balance_pitch_pid_controller.getP())
        self.sd.putNumber('Balance Pitch kI', self.balance_pitch_pid_controller.getI())
        self.sd.putNumber('Balance Pitch kD', self.balance_pitch_pid_controller.getD())

        self.balance_yaw_pid_controller = PIDController(self.balance_config.balance_yaw_kP, self.balance_config.balance_yaw_kI, self.balance_config.balance_yaw_kD)
        self.balance_yaw_pid_controller.enableContinuousInput(0, 360)
        self.balance_yaw_pid_controller.setTolerance(0.5, 0.5) # may need to tweak this with PID testing

        self.sd.putNumber('Balance Yaw kP', self.balance_yaw_pid_controller.getP())
        self.sd.putNumber('Balance Yaw kI', self.balance_yaw_pid_controller.getI())
        self.sd.putNumber('Balance Yaw kD', self.balance_yaw_pid_controller.getD())

        self.target_config = _target_cfg
        self.target_kP = self.target_config.target_kP
        self.target_kI = self.target_config.target_kI
        self.target_kD = self.target_config.target_kD
        self.target_x_pid_controller = PIDController(self.target_config.target_kP, self.target_config.target_kI, self.target_config.target_kD)
        self.target_x_pid_controller.setTolerance(5, 5)
        self.target_y_pid_controller = PIDController(self.target_config.target_kP, self.target_config.target_kI, self.target_config.target_kD)
        self.target_y_pid_controller.setTolerance(5, 5)
        self.target_rcw_pid_controller = PIDController(self.target_config.target_kP, self.target_config.target_kI, self.target_config.target_kD)
        self.target_rcw_pid_controller.setTolerance(0.5, 0.5)
        self.target_rcw_pid_controller.enableContinuousInput(0, 360)

        self.bearing_config = _bearing_cfg
        self.bearing_kP = self.bearing_config.bearing_kP
        self.bearing_kI = self.bearing_config.bearing_kI
        self.bearing_kD = self.bearing_config.bearing_kD
        self.bearing_pid_controller = PIDController(self.bearing_kP, self.bearing_kI, self.bearing_kD)

        self.bearing = self.getGyroAngle()
        self.updateBearing = False
        
    def reset(self):
        print("In swervedrive reset")

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
        print("Angle: ", self.getGyroAngle(), ", Pitch: ", self.getGyroPitch(), ", Yaw: ", self.getGyroYaw(), ", Roll: ", self.getGyroRoll())

    def resetGyro(self):
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
        
        self.balance_pitch_pid_controller.setP(self.sd.getNumber('Balance Pitch kP', 0))
        self.balance_pitch_pid_controller.setI(self.sd.getNumber('Balance Pitch kI', 0))
        self.balance_pitch_pid_controller.setD(self.sd.getNumber('Balance Pitch kD', 0))

        #print("Pitch: kP = ", self.sd.getNumber('Balance Pitch kP', 0), ", kI = ", self.sd.getNumber('Balance Pitch kI', 0), ", kD =", self.sd.getNumber('Balance Pitch kD', 0))
        
        self.balance_yaw_pid_controller.setP(self.sd.getNumber('Balance Yaw kP', 0))
        self.balance_yaw_pid_controller.setI(self.sd.getNumber('Balance Yaw kI', 0))
        self.balance_yaw_pid_controller.setD(self.sd.getNumber('Balance Yaw kD', 0))

        #print("Yaw: kP = ", self.sd.getNumber('Balance Yaw kP', 0), ", kI = ", self.sd.getNumber('Balance Yaw kI', 0), ", kD =", self.sd.getNumber('Balance Yaw kD', 0))
        
        #self.printGyro()

        yawSign = -1

        if(self.getGyroYaw() >= -90 and self.getGyroYaw() <= 90):
            BALANCED_YAW = 0.0
            yawSign = +1
        else:
            BALANCED_YAW = 180.0
            yawSign = -1
        BALANCED_PITCH = 0.0

        #print("Yaw = ", self.getGyroYaw(), " BALANCED_YAW = ", BALANCED_YAW, " BALANCED_PITCH = ", BALANCED_PITCH)

        pitch_error = self.balance_pitch_pid_controller.calculate(self.getGyroBalance(), BALANCED_PITCH) 
        yaw_error = self.balance_yaw_pid_controller.calculate(self.getGyroYaw(), BALANCED_YAW) 

        # Set the output to 0 if at setpoint or to a value between (-1, 1)
        if self.balance_pitch_pid_controller.atSetpoint():
            pitch_output = 0
        else:
            #pitch_output = clamp(pitch_error)
            pitch_output = pitch_error

        if self.balance_yaw_pid_controller.atSetpoint():
            yaw_output = 0
        else:
            yaw_output = clamp(yaw_error)
        
        print("Pitch setpoint: ", self.balance_pitch_pid_controller.getSetpoint(), "pitch output: ", pitch_output, " pitch error: ", pitch_error)
        print("Yaw setpoint: ", self.balance_yaw_pid_controller.getSetpoint(), "yaw output: ", yaw_output, " yaw error: ", yaw_error)

        # Put the output to the dashboard
        self.sd.putNumber('Balance pitch output', pitch_output)
        self.sd.putNumber('Balance yaw output', yaw_output)
        self.move(yawSign * pitch_output, 0.0, yaw_output)
        
        self.update_smartdash()

        self.execute()

        if self.balance_pitch_pid_controller.atSetpoint() and self.balance_yaw_pid_controller.atSetpoint():
            return True
        else:
            return False

    def steerStraight3(self, rcw):

        current_angle = self.getGyroAngle()
        if rcw != 0:
            self.updateBearing = True
            print("rcw (!=0): ", rcw, " bearing: ", self.bearing, " currentAngle: ", current_angle)
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
            print("rcw: ", rcw, " rcw_error: ", rcw_error, " current_angle: ", current_angle, " bearing: ", self.bearing, " target_angle: ", target_angle)
            return rcw_error


    def steerStraight2(self, rcw):
        if rcw != 0:
            self.updateBearing = True
            return self.getGyroAngle() % 360
        else:
            self.updateBearing = False
            return self.bearing
    
    def steerStraight(self, rcw):
        if rcw != 0:
            self.updateBearing = True
            print("rcw (!=0): ", rcw, " bearing: ", self.bearing, " currentAngle: ", self.getGyroAngle())
            return rcw
        else:
            self.updateBearing = False
            angle_diff = abs(self.getGyroAngle() - self.bearing)
            if (angle_diff) < 180:
                print("Angle Diff: ", angle_diff, " GyroAngle: ", self.getGyroAngle(), " bearing: ", self.bearing)
                rcw_error = self.bearing_pid_controller.calculate(self.getGyroAngle(), self.bearing)
            else:
                print("Angle Diff: ", angle_diff, " 360 - GyroAngle: ", (360 - self.getGyroAngle()), " bearing: ", self.bearing)
                rcw_error = -self.bearing_pid_controller.calculate((360 - self.getGyroAngle()), self.bearing)
            print("rcw: ", rcw, " rcw_error: ", rcw_error, " currentAngle: ", self.getGyroAngle(), " bearing: ", self.bearing)
            return rcw_error

    def move(self, non_adjusted_fwd, non_adjusted_strafe, rcw):
        """
        Calulates the speed and angle for each wheel given the requested movement
        Positive fwd value = Forward robot movement\n
        Negative fwd value = Backward robot movement\n
        Positive strafe value = Left robot movement\n
        Negative strafe value = Right robot movement
        :param fwd: the requested movement in the Y direction 2D plane
        :param strafe: the requested movement in the X direction of the 2D plane
        :param rcw: the requestest magnatude of the rotational vector of a 2D plane
        """

        fwd = non_adjusted_fwd * self.swervometer.getTeamMoveAdjustment()
        strafe = non_adjusted_strafe * self.swervometer.getTeamMoveAdjustment()

        #Convert field-oriented translate to chassis-oriented translate
        
        #current_angle = self.steerStraight2(rcw)
        current_angle = self.getGyroAngle() % 360
        desired_angle = ((math.atan2(fwd, strafe) / math.pi) * 180) % 360
        chassis_angle = (desired_angle - current_angle) % 360
        magnitude = clamp(math.hypot(fwd, strafe), 0, 1)

        chassis_strafe = magnitude * math.sin(math.radians(chassis_angle))
        chassis_fwd = magnitude * math.cos(math.radians(chassis_angle))

        #print("modified strafe: " + str(chassis_strafe) + ", modified fwd: " + str(chassis_fwd))
        self.sd.putNumber("Current Gyro Angle", self.getGyroAngle())

        self.set_fwd(chassis_fwd)
        self.set_strafe(chassis_strafe)

        # self.set_fwd(fwd)
        # self.set_strafe(strafe)
        rcw = self.steerStraight3(rcw)
        self.set_rcw(rcw)
    
    def goToPose(self, x, y, rcw):

        currentX, currentY, currentRCW = self.swervometer.getCOF()
        x_error = -self.target_x_pid_controller.calculate(currentX, x)
        y_error = self.target_y_pid_controller.calculate(currentY, y)

        if rcw != 0:
            rcw_error = self.target_rcw_pid_controller.calculate(currentRCW, rcw)
        else:
            rcw_error = self.steerStraight3(rcw)
        
        #print("hello: x: ", self.target_x_pid_controller.getSetpoint(), " y: ", self.target_y_pid_controller.getSetpoint())
        if self.target_x_pid_controller.atSetpoint():
            print("X at set point")
        if self.target_y_pid_controller.atSetpoint():
            print("Y at set point")
        if self.target_rcw_pid_controller.atSetpoint():
            print("RCW at set point")
            
        if self.target_x_pid_controller.atSetpoint() and self.target_y_pid_controller.atSetpoint() and self.target_rcw_pid_controller.atSetpoint(): 
            self.update_smartdash()
            return True
        else:
            self.move(x_error, y_error, rcw_error)
            self.update_smartdash()
            self.execute()
            print("xPositionError: ", self.target_x_pid_controller.getPositionError(), "yPositionError: ", self.target_y_pid_controller.getPositionError(), "rcwPositionError: ", self.target_rcw_pid_controller.getPositionError())
            # print("xPositionTolerance: ", self.target_x_pid_controller.getPositionTolerance(), "yPositionTolerance: ", self.target_y_pid_controller.getPositionTolerance(), "rcwPositionTolerance: ", self.target_rcw_pid_controller.getPositionTolerance())
            print("currentX: ", currentX, " targetX: ", x, "x_error: ", x_error, " currentY: ", currentY, " targetY: ", y, " y_error: ", y_error, " currentRCW: ", currentRCW, " targetRCW: ", rcw, " rcw_error: ", rcw_error)
            return False

    def _calculate_vectors(self):
        """
        Calculate the requested speed and angle of each modules from self._requested_vectors and store them in
        self._requested_speeds and self._requested_angles dictionaries.
        """
        print("in _calculate_vectors")
        self._requested_vectors['fwd'], self._requested_vectors['strafe'], self._requested_vectors['rcw'] = self.normalize([self._requested_vectors['fwd'], self._requested_vectors['strafe'], self._requested_vectors['rcw']])

        # Does nothing if the values are lower than the input thresh
        if self.threshold_input_vectors:
            #print("checking thresholds: fwd: ", self._requested_vectors['fwd'], "strafe: ", self._requested_vectors['strafe'], "rcw: ", self._requested_vectors['rcw'])
            if abs(self._requested_vectors['fwd']) < self.lower_input_thresh:
                #print("forward = 0")
                self._requested_vectors['fwd'] = 0

            if abs(self._requested_vectors['strafe']) < self.lower_input_thresh:
                #print("strafe = 0")
                self._requested_vectors['strafe'] = 0

            if abs(self._requested_vectors['rcw']) < self.lower_input_thresh:
                #print("rcw = 0")
                self._requested_vectors['rcw'] = 0

            if self._requested_vectors['rcw'] == 0 and self._requested_vectors['strafe'] == 0 and self._requested_vectors['fwd'] == 0:  # Prevents a useless loop.
                #print("all three zero")
                self._requested_speeds = dict.fromkeys(self._requested_speeds, 0) # Do NOT reset the wheel angles.

                if self.wheel_lock:
                    # This is intended to set the wheels in such a way that it
                    # difficult to push the robot (intended for defence)

                    self._requested_angles['front_left'] = 45
                    self._requested_angles['front_right'] = -45
                    self._requested_angles['rear_left'] = -45
                    self._requested_angles['rear_right'] = 45

                    #self.wheel_lock = False
                    #print("testing wheel lock")
                return
        
        frame_dimension_x, frame_dimension_y = self.swervometer.getFrameDimensions()
        ratio = math.hypot(frame_dimension_x, frame_dimension_y)

        # Velocities per quadrant
        frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
        rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (frame_dimension_y / ratio))
        leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))
        rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (frame_dimension_x / ratio))

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

    def setWheelLock(self, isLocked):
        #print("is locked", isLocked)
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
        
        print('Requested values: ', self._requested_vectors, '\n')
        print('Requested angles: ', self._requested_angles, '\n')
        print('Requested speeds: ', self._requested_speeds, '\n')

    def execute(self):
        """
        Sends the speeds and angles to each corresponding wheel module.
        Executes the doit in each wheel module.
        """
        self.update_smartdash()

        # Calculate each vector
        self._calculate_vectors()

        # Set the speed and angle for each module

        # Calculate normalized speeds with lever arm adjustment
        for key in self.modules:
            #print("Execute: key: ", key, " base speed: ", self._requested_speeds[key], " COMmult: ", self.swervometer.getCOMmult(key), " adjusted speed: ", (self._requested_speeds[key] * self.swervometer.getCOMmult(key)), self._requested_speeds[key] * self.swervometer.getCOMmult(key))
            self._requested_speeds[key] = self._requested_speeds[key] * self.swervometer.getCOMmult(key)
        
        self._requested_speeds = self.normalizeDictionary(self._requested_speeds)

        for key in self.modules:
            self.modules[key].move(self._requested_speeds[key], self._requested_angles[key])
        
        # Reset the speed back to zero
        self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)

        # Execute each module
        first_module = True
        for key in self.modules:
            print("Module: Key: ", key)
            self.modules[key].execute()

        COFX, COFY, COFAngle = self.swervometer.calculateCOFPose(self.modules, self.getGyroAngle())

        if self.vision:
            print("Vision started")
            if self.vision.canUpdatePose():
                print("Vision: canupdatepose")
                pose = self.vision.getPose()
                orientation = self.vision.getOrientation()
                if self.vision.shouldUpdatePose():
                    if pose[0] != -1:
                        self.swervometer.setCOF(pose[0], pose[1], orientation[2])
                        print("Vision updated position: (" + str(pose[0]) + ", " + str(pose[1]) + ") with rotation of " + str(orientation[2]) + " degrees.")
                    else:
                        print("Vision should have updated position, but pose was empty.")
                else:
                    print("Vision reports position: (" + str(pose[0]) + ", " + str(pose[1]) + ") with rotation of " + str(orientation[2]) + " degrees.")
                print("AFTER COMMENTS")

        print("COFX: ", COFX, ", COFY: ", COFY, ", COF Angle: ", COFAngle)

        if(self.updateBearing):
            print("Old Bearing: ", self.bearing)
            self.bearing = self.getGyroAngle()
            print("New Bearing: ", self.bearing)
            self.updateBearing = False

    def idle(self):
        for key in self.modules:
            self.modules[key].idle()
            
    def update_smartdash(self):
        """
        Pushes some internal variables for debugging.
        """
        if self.debugging:
            for key in self._requested_angles:
                self.sd.putNumber('drive/drive/%s_angle' % key, self._requested_angles[key])
                self.sd.putNumber('drive/drive/%s_speed' % key, self._requested_speeds[key])
                
