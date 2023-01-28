import wpilib
from collections import namedtuple

# Create the structure of the field config:
FieldConfig = namedtuple('FieldConfig', ['sd_prefix',
                                         'origin_x', 'origin_y',
                                         'start_position_x', 'start_position_y',
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
                                                         'gyro_offset_x', 'gyro_offset_y',
                                                         'camera_offset_x', 'camera_offset_y'])
