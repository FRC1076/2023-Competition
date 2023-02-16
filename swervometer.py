import math
import wpilib
from collections import namedtuple

# Create the structure of the field config:
FieldConfig = namedtuple('FieldConfig', ['sd_prefix',
                                         'origin_x', 'origin_y',
                                         'start_position_x', 'start_position_y', 'start_angle',
                                         'tag1_x', 'tag1_y',
                                         'tag2_x', 'tag2_y',
                                         'tag3_x', 'tag3_y',
                                         'tag4_x', 'tag4_y',
                                         'tag5_x', 'tag5_y',
                                         'tag6_x', 'tag6_y',
                                         'tag7_x', 'tag7_y',
                                         'tag8_x', 'tag8_y'])

# Create the structure of the robot property config: 
RobotPropertyConfig = namedtuple('RobotPropertyConfig', ['sd_prefix',
                                                         'is_red_team',
                                                         'frame_dimension_x', 'frame_dimension_y',
                                                         'bumper_dimension_x', 'bumper_dimension_y',
                                                         'com_offset_x', 'com_offset_y',
                                                         'gyro_offset_x', 'gyro_offset_y',
                                                         'camera_offset_x', 'camera_offset_y',
                                                         'swerve_module_offset_x', 'swerve_module_offset_y'])

class Swervometer:
    def __init__(self, field_cfg, robot_property_cfg):
        self.field = field_cfg
        self.robotProperty = robot_property_cfg
        self.currentX = self.field.origin_x + self.field.start_position_x + self.robotProperty.bumper_dimension_x + self.robotProperty.com_offset_x
        self.currentY = self.field.origin_y + self.field.start_position_y + self.robotProperty.bumper_dimension_y + self.robotProperty.com_offset_y
        self.currentRCW = self.field.start_angle
        self.swerveModuleOffsetX = self.robotProperty.swerve_module_offset_x
        self.swerveModuleOffsetY = self.robotProperty.swerve_module_offset_y
        self.movingToTarget = False
        print("init current X: ", self.currentX, " init current y: ", self.currentY)
    
    def getCOF(self):
        return self.currentX, self.currentY, self.currentRCW

    def setCOF(self, x, y, rcw):
        self.currentX = x
        self.currentY = y
        self.currentRCW = rcw

    def calculateModuleCoordinates(self, psi, currentGyroAngle, hypotenuse, positionChange, wheelAngle):
        baseAngle = (psi + currentGyroAngle) % 360 # angle of the module
        swerveModuleOffsetXCoordinate = hypotenuse * math.sin(baseAngle) # X-position of the module
        swerveModuleOffsetYCoordinate = hypotenuse * math.cos(baseAngle) # Y-position of the module
        
        combinedAngle = (baseAngle + wheelAngle) % 360 # angle of the wheel
        XChange = positionChange * math.sin(combinedAngle) # change in X-position of the module
        YChange = positionChange * math.cos(combinedAngle) # change in Y-position of the module
        
        XCoordinate = self.currentX + swerveModuleOffsetXCoordinate + XChange # current X-coordinate of COF plus swerve module offset plus movement
        YCoordinate = self.currentY + swerveModuleOffsetYCoordinate + YChange # current Y-coordinate of COF plus swerve module offset plus movement

        return XCoordinate, YCoordinate

    def calculateCOFPose(self, modules, currentGyroAngle):

        # The bot is assumed to orient so that "0 degrees" faces "north" along the Y-axis of the frame in the forward direction.
        
        # currentGyroAngle is the clockwise rotation of the bot as determined by the gyro.

        # psi is the fixed angle from the front of the bot to the front right corner.
        # Mod 360 shouldn't be needed.
        # Although we recalculate it here, each psi and the hypotenuse are constants.

        frontRightPsi = math.atan(self.swerveModuleOffsetX / self.swerveModuleOffsetY) % 360
        rearRightPsi = (frontRightPsi + 90) % 360
        rearLeftPsi = (frontRightPsi + 180) % 360
        frontLeftPsi = (frontRightPsi + 270) % 360
        hypotenuse = math.sqrt((self.swerveModuleOffsetX ** 2) + (self.swerveModuleOffsetY ** 2))
        
        for key in modules:
            # positionChange is the amount the wheel moved forward
            positionChange = modules[key].positionChange

            # wheelAngle is the angle of the module wheel relative to the frame of the bot
            wheelAngle = (modules[key].newAngle + 0) % 360
            
            # Each of these calculations is different because positionChange, newAngle, and psi are different for each corner
            if (key == 'front_right'):
                frontRightXCoordinate, frontRightYCoordinate = self.calculateModuleCoordinates(frontRightPsi, currentGyroAngle, hypotenuse, positionChange, wheelAngle)
                print("fr: pc: ", positionChange, " psi: ", frontRightPsi, " bot angle: ", currentGyroAngle, " wheel angle: ", wheelAngle, "frx: ", frontRightXCoordinate, "fry: ", frontRightYCoordinate)
            elif (key == 'rear_right'):
                rearRightXCoordinate, rearRightYCoordinate = self.calculateModuleCoordinates(rearRightPsi, currentGyroAngle, hypotenuse, positionChange, wheelAngle)
                print("rr: pc: ", positionChange, " psi: ", rearRightPsi, " bot angle: ", currentGyroAngle, " wheel angle: ", wheelAngle, "rrx: ", rearRightXCoordinate, "rry: ", rearRightYCoordinate)
            elif (key == 'rear_left'):
                rearLeftXCoordinate, rearLeftYCoordinate = self.calculateModuleCoordinates(rearLeftPsi, currentGyroAngle, hypotenuse, positionChange, wheelAngle)
                print("rl: pc: ", positionChange, " psi: ", rearLeftPsi, " bot angle: ", currentGyroAngle, " wheel angle: ", wheelAngle, "rlx: ", rearLeftXCoordinate, "rly: ", rearLeftYCoordinate)
            else: # (key == 'front_left'):
                frontLeftXCoordinate, frontLeftYCoordinate = self.calculateModuleCoordinates(frontLeftPsi, currentGyroAngle, hypotenuse, positionChange, wheelAngle)
                print("fl: pc: ", positionChange, " psi: ", frontLeftPsi, " bot angle: ", currentGyroAngle, " wheel angle: ", wheelAngle, "flx: ", frontLeftXCoordinate, "fly: ", frontLeftYCoordinate)
            
        # Find average COF XY-coordinates of bot
        midpointX1 = (frontLeftXCoordinate + rearRightXCoordinate)/2
        midpointY1 = (frontLeftYCoordinate + rearRightYCoordinate)/2

        midpointX2 = (frontRightXCoordinate + rearLeftXCoordinate)/2
        midpointY2 = (frontRightYCoordinate + rearLeftYCoordinate)/2

        midpointX = (midpointX1 + midpointX2)/2
        midpointY = (midpointY1 + midpointY2)/2

        print("old x: ", self.currentX, " old y: ", self.currentY)
        print("draftchange x1:", midpointX1, "draftchange x2:", midpointX2, "change x: ", midpointX, "draftchange y1:", midpointY1, "draftchange y2:", midpointY2, " change y: ", midpointY)

        # Reset pose of bot.
        self.currentX = midpointX
        self.currentY = midpointY
        self.currentRCW = currentGyroAngle
        
        return self.currentX, self.currentY, self.currentRCW
    
    def setMovingToTarget(self, _movingToTarget):
        self.movingToTarget = _movingToTarget

    def setTarget(self, x, y, rcw):
        self.targetX = x
        self.targetY = y
        self.targetRCW = rcw

    def getTarget(self):
        return self.targetX, self.targetY, self.targetRCW

    def setPositionTuple(self, x, y, rcw):
        self.currentX = x
        self.currentY = y
        self.currentRCW = rcw

    def getPositionTuple(self):
        return self.currentX, self.currentY, self.currentRCW
        
    def updatePositionTupleFromCamera(self, tagNumber, relativePositionTuple):
        tagNum = tagNumber
        x,y,rcw = relativePositionTuple

        tagX, tagY = self.getTagPosition(tagNum)

        self.currentX = tagX - x - (self.field.camera_offset_x - self.field.com_offset_x)
        self.currentY = tagY - y - (self.field.camera_offset_y - self.field.com_offset_y)
        self.currentRCW = rcw
        return self.currentX, self.currentY, self.currentRCW

    def getTagPosition(self, tagNumber):
        match tagNumber:
            case 1:
                return self.field.tag1_x, self.field.tag1_y
            case 2:
                return self.field.tag2_x, self.field.tag2_y
            case 3:
                return self.field.tag3_x, self.field.tag3_y
            case 4:
                return self.field.tag4_x, self.field.tag4_y
            case 5:
                return self.field.tag5_x, self.field.tag5_y
            case 6:
                return self.field.tag6_x, self.field.tag6_y
            case 7:
                return self.field.tag7_x, self.field.tag7_y
            case 8:
                return self.field.tag8_x, self.field.tag8_y
            case _: # Should never happen
                print("Unknown tag found.")
                return -1, -1