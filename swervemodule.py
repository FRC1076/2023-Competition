import math
from util import clamp

import wpilib
import wpilib.drive
import wpimath.controller
import ctre
import rev

from networktables import NetworkTables
from wpimath.controller import PIDController
from collections import namedtuple

from dashboard import Dashboard
from robotconfig import MODULE_NAMES

# Create the structure of the config: SmartDashboard prefix, Encoder's zero point, Drive motor inverted, Allow reverse
ModuleConfig = namedtuple('ModuleConfig', ['sd_prefix', 'zero', 'inverted', 'allow_reverse', 'position_conversion', 'heading_kP', 'heading_kI', 'heading_kD'])

MAX_VOLTAGE = 5 # Absolute encoder measures from 0V to 5V
DASH_PREFIX = MODULE_NAMES.SWERVEMODULE

class SwerveModule:

    def __init__(
            self, 
            _driveMotor, 
            _driveEncoder, 
            _rotateMotor, 
            _rotateEncoder, 
            _config, 
        ):
        
        self.cfg = _config

        self.rotateMotor = _rotateMotor
        self.rotateEncoder = _rotateEncoder

        self.driveMotor = _driveMotor
        self.driveEncoder = _driveEncoder

        self.driveEncoder.setPosition(0)
        self.driveEncoder.setPositionConversionFactor(self.cfg.position_conversion)
        self.lastPosition = self.driveEncoder.getPosition()

        self.encoder_zero = self.cfg.zero or 0 #idk the point of this, maybe useful for other encoder type
        angle = ((self.rotateEncoder.getAbsolutePosition() - self.encoder_zero) % 360)
        if angle >= 90 and angle <= 270:
            self.positionSign = +1 
            self.moduleFlipped = False
            #print("SwerveModule Init (90 <= theta <= 270): positionSign: ", self.positionSign, " angle: ", angle)
        else:
            self.positionSign = -1
            self.moduleFlipped = True
            #print("SwerveModule Init (theta < 90 or theta > 270): positionSign: ", self.positionSign, " angle: ", angle)

        # Config -- change this to reflect how our config is formatted. We will upon testing of the entire drivetrain figure out which need to be inverted.
        self.sd_prefix = self.cfg.sd_prefix or 'Module'
        self.inverted = self.cfg.inverted or False 
        self.allow_reverse = self.cfg.allow_reverse or True #def allow reverse always, so you can maybe remove this

        # SmartDashboard
        self.sd = Dashboard.getDashboard()
        self.debugging = False # should get rid of this

        # Motor
        self.driveMotor.setInverted(self.inverted)
        self.rotateMotor.setInverted(False)

        self._requested_angle = 0 # change this to something like 'requested angle' or 'requested encoder value', whatever makes more sense
        self._requested_speed = 0 #class variable which execute() passes to the drive motor at the end of the robot loop

        # Heading PID Controller
        # kP = 1.5, kI = 0.0, kD = 0.0
        self.heading_pid_controller = PIDController(self.cfg.heading_kP, self.cfg.heading_kI, self.cfg.heading_kD) #swap this stuff for CANSparkMax pid controller -- see example from last year shooter
        self.heading_pid_controller.enableContinuousInput(0, 360)
        self.heading_pid_controller.setTolerance(0.5, 0.5) # may need to tweak this with PID testing

        self.sd.putNumber(DASH_PREFIX, 'Heading kP', self.heading_pid_controller.getP())
        self.sd.putNumber(DASH_PREFIX, 'Heading kI', self.heading_pid_controller.getI())
        self.sd.putNumber(DASH_PREFIX, 'Heading kD', self.heading_pid_controller.getD())

    def reset(self):
        
        #print("In swervemodule reset")

        self.driveEncoder.setPosition(0)
        self.driveEncoder.setPositionConversionFactor(self.cfg.position_conversion)
        self.lastPosition = self.driveEncoder.getPosition()

        #print("in module reset: position: ", self.lastPosition * self.positionSign)

        #angle = (self.rotateEncoder.getAbsolutePosition() - self.encoder_zero) % 360
        #self.moduleFlipped = False
        #if angle >= 90 and angle <= 270:
        #    self.positionSign = 1
            
        #    print("SwerveModule Reset: positionSign: ", self.positionSign, " angle: ", angle)
        #else:
        #    self.positionSign = -1
        #    print("SwerveModule Reset: positionSign: ", self.positionSign, " angle: ", angle)

        
        # Motor
        #self.driveMotor.setInverted(self.inverted)
        #self.rotateMotor.setInverted(False)

        #self._requested_angle = 0 # change this to something like 'requested angle' or 'requested encoder value', whatever makes more sense
        #self._requested_speed = 0 #class variable which execute() passes to the drive motor at the end of the robot loop

    def get_current_velocity(self):
        velocity = self.driveEncoder.getVelocity()
        #multiply by ratio (inches / rotation)
        return velocity

    def get_current_angle(self):
        """
        :returns: the voltage position after the zero
        """
        angle = (self.rotateEncoder.getAbsolutePosition() - self.encoder_zero) % 360
        
        if self.moduleFlipped:
            angle = (angle + 180) % 360
            #self.positionSign *= -1


        return angle

    def flush(self): # rewrite this (although it isnt used anywhere) to reset the encoder to 0 and zero out the speed, if you want.
        """
        Flush the modules requested speed and voltage.
        Resets the PID controller.
        """
        self._requested_angle = 0
        self._requested_speed = 0
        self.heading_pid_controller.reset()

    
    @staticmethod
    def voltage_to_degrees(voltage): #make this like TICKS TO DEG or smth
        """
        Convert a given voltage value to degrees.
        :param voltage: a voltage value between 0 and 5
        :returns: the degree value between 0 and 359
        """
        deg = (voltage / 5) * 360

        if deg < 0:
            deg += 360

        return deg

    @staticmethod
    def voltage_to_rad(voltage): #same, but TICKS TO RAD
        """
        Convert a given voltage value to rad.
        :param voltage: a voltage value between 0 and 5
        :returns: the radian value betwen 0 and 2pi
        """
        return (voltage / 5) * 2 * math.pi

    @staticmethod
    def degree_to_voltage(degree): #DEG TO TICK
        """
        Convert a given degree to voltage.
        :param degree: a degree value between 0 and 360
        :returns" the voltage value between 0 and 5
        """
        return (degree / 360) * 5

    def _set_deg(self, value): #This one weird. Dont do mod stuff if possible - it messes up PID calculations
        """
        Round the value to within 360. Set the requested rotate position (requested voltage).
        Intended to be used only by the move function.
        """
        self._requested_angle = value % 360

    def move(self, speed, deg): #this is all good mostly
        """
        Set the requested speed and rotation of passed.
        :param speed: requested speed of wheel from -1 to 1
        :param deg: requested angle of wheel from 0 to 359 (Will wrap if over or under)
        """
        # deg %= 360 # mod 360, may want to change
        
        
        if self.allow_reverse: #addresses module-flipping
            """
            If the difference between the requested degree and the current degree is
            more than 90 degrees, don't turn the wheel 180 degrees. Instead reverse the speed.
            """
            diff = abs(deg - self.get_current_angle())

            if (diff > 180):
                diff = 360 - diff

            if diff > 90: #make this with the new tick-degree methods
                self.moduleFlipped = not self.moduleFlipped
                self.positionSign *= -1

            if self.moduleFlipped:
                speed *= -1
                #deg += 180
                #deg %= 360
            
            #print("Module Flipped Test: flipped: ", self.moduleFlipped, " speed: ", speed, " positionSign: ", self.positionSign)

        self._requested_speed = speed 
        self._set_deg(deg)
        #print("speed:", speed, " degree:", deg)

    def debug(self): #can use logging/SD if useful
        """
        Print debugging information about the module to the log.
        """
        print(self.sd_prefix, '; requested_speed: ', self._requested_speed, ' requested_angle: ', self._requested_angle)

    def execute(self):
        """
        Use the PID controller to get closer to the requested position.
        Set the speed requested of the drive motor.
        Called every robot iteration/loop.
        """

        self.heading_pid_controller.setP(self.sd.getNumber(DASH_PREFIX, 'Heading kP', 0))
        self.heading_pid_controller.setI(self.sd.getNumber(DASH_PREFIX, 'Heading kI', 0))
        self.heading_pid_controller.setD(self.sd.getNumber(DASH_PREFIX, 'Heading kD', 0))

        # Calculate the error using the current voltage and the requested voltage.
        # DO NOT use the #self.get_voltage function here. It has to be the raw voltage.
        error = self.heading_pid_controller.calculate(self.get_current_angle(), self._requested_angle) #Make this an error in ticks instead of voltage
 
        # Set the output 0 as the default value
        output = 0
        # If the error is not tolerable, set the output to the error.

        # Else, the output will stay at zero.
        if not self.heading_pid_controller.atSetpoint():
            # Use max-min to clamped the output between -1 and 1. The CANSparkMax PID controller does this automatically, so idk if this is necessary
            output = clamp(error)

        #print('ERROR = ' + str(error) + ', OUTPUT = ' + str(output))

        # Put the output to the dashboard
        self.sd.putNumber(DASH_PREFIX, '/%s/rotation_output' % self.sd_prefix, output)
        # Set the output as the rotateMotor's voltage
        self.rotateMotor.set(output) # will replace this with a set SETPOINT rather than actually setting the speed
        #SparkMax PID controller will take care of actually running the motors with PID values you instantiate it with

        # Set the requested speed as the driveMotor's voltage
        self.driveMotor.set(self._requested_speed)

        #print("Angle: ", self.get_current_angle(), " Absolute Position: ", self.sd_prefix, " ", self.encoder.getAbsolutePosition(), self.encoder_zero, self.encoder.getAbsolutePosition() - self.encoder_zero)

        self.newPosition = self.driveEncoder.getPosition() * 1.79
        #print("FACTOR: ", self.driveEncoder.getPositionConversionFactor())
        self.positionChange = (self.newPosition - self.lastPosition) * self.positionSign
        #print("Position Change: ", self.positionChange, " New: ", self.newPosition, " Last: ", self.lastPosition, " Sign: ", self.positionSign)
        self.newAngle = self.get_current_angle()
        self.lastPosition = self.newPosition # save it for next time

        self.update_smartdash()
    
    def testMove(self, driveInput, rotateInput):
        self.driveMotor.set(clamp(driveInput))
        self.rotateMotor.set(clamp(rotateInput))

    def idle(self):
        self.rotateMotor.set(0)
        self.driveMotor.set(0)

    def setRampRate(self, openLoopRampRate, closedLoopRampRate):
        self.driveMotor.setOpenLoopRampRate(openLoopRampRate)
        print("Open Loop Ramp Rate: ", self.driveMotor.getOpenLoopRampRate())
        self.driveMotor.setClosedLoopRampRate(closedLoopRampRate)
        print("Closed Loop Ramp Rate: ", self.driveMotor.getClosedLoopRampRate())

    def update_smartdash(self):
        """
        Output a bunch on internal variables for debugging purposes.
        """
        self.sd.putNumber(DASH_PREFIX, '/%s/current_angle' % self.sd_prefix, self.get_current_angle())

        # if self.debugging.getBoolean(False):
        # not sure why we need this or how it gets set in the first place

        self.sd.putNumber(DASH_PREFIX, '/%s/requested_speed' % self.sd_prefix, self._requested_speed)
        self.sd.putNumber(DASH_PREFIX, '/%s/requested_angle' % self.sd_prefix, self._requested_angle)
        self.sd.putNumber(DASH_PREFIX, '/%s/rotate encoder absolute position' % self.sd_prefix, self.rotateEncoder.getPosition())
        self.sd.putNumber(DASH_PREFIX, '/%s/drive encoder absolute position' % self.sd_prefix, self.driveEncoder.getPosition())

        self.sd.putNumber(DASH_PREFIX, '/%s/Heading PID Setpoint' % self.sd_prefix, self.heading_pid_controller.getSetpoint())
        self.sd.putNumber(DASH_PREFIX, '/%s/Heading PID Error' % self.sd_prefix, self.heading_pid_controller.getPositionError())
        self.sd.putBoolean(DASH_PREFIX, '/%s/Heading PID isAligned' % self.sd_prefix, self.heading_pid_controller.atSetpoint())

        self.sd.putBoolean(DASH_PREFIX, '/%s/allow_reverse' % self.sd_prefix, self.allow_reverse)
        self.sd.putBoolean(DASH_PREFIX, '/%s/is_flipped' % self.sd_prefix, self.moduleFlipped)
        self.sd.putNumber(DASH_PREFIX, '/%s/encoder_zero' % self.sd_prefix, self.encoder_zero)
