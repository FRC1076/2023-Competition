import math
#import numpy
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
                                                         'swerve_module_offset_x', 'swerve_module_offset_y',
                                                         'inches_per_rotation'])

class Swervometer:
    def __init__(self, field_cfg, robot_property_cfg):
        self.field = field_cfg
        self.robotProperty = robot_property_cfg
        self.currentX = self.field.origin_x + self.field.start_position_x + self.robotProperty.bumper_dimension_x + self.robotProperty.com_offset_x
        self.currentY = self.field.origin_y + self.field.start_position_y + self.robotProperty.bumper_dimension_y + self.robotProperty.com_offset_y
        self.currentAngle = self.field.start_angle
        self.inchesPerRotation = self.robotProperty.inches_per_rotation
        self.swerveModuleOffsetX = self.robotProperty.swerve_module_offset_x
        self.swerveModuleOffsetY = self.robotProperty.swerve_module_offset_y
        
    def getPositionTuple(self):
        return self.currentX, self.currentY, self.currentAngle
        
    def updatePositionTupleFromWheels(self, x, y, rcw):
        self.currentX += x
        self.currentY += y
        self.currentAngle = rcw
        return self.currentX, self.currentY, self.currentAngle

    def calcTranslationalAndRotationalXandY(self, x_input, y_input, rcw, old_heading, distance, frameX, frameY):
        
        # theta is the angle from zero that robot heads without rotation
        theta = numpy.arctan(x_input / y_input)

        # find the amount of translational movement
        x_translational = distance * numpy.sin(theta)
        y_translational = distance * numpy.cos(theta)

        # r is the radius of the frame
        r = math.sqrt((frameX * frameX) + (frameY + frameY))

        # psi is the angle __
        psi = (rcw + old_heading) % 360

        # find the amount of rotational movement
        rotational_distance = r * numpy.tan(psi)
        x_rotational = rotational_distance * numpy.cos(psi)
        y_rotational = rotational_distance * numpy.sin(psi)

        return x_translational, y_translational, x_rotational, y_rotational

    def updatePositionTupleFromCamera(self, tagNumber, relativePositionTuple):
        tagNum = tagNumber
        x,y,rcw = relativePositionTuple

        tagX, tagY = self.getTagPosition(tagNum)

        self.currentX = tagX - x - (self.field.camera_offset_x - self.field.com_offset_x)
        self.currentY = tagY - y - (self.field.camera_offset_y - self.field.com_offset_y)
        self.currentAngle = rcw
        return self.currentX, self.currentY, self.currentAngle

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