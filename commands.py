
from robot import MyRobot

# Base action, so that the command processor can interact with all Actions the same way
class Action:
    def __init__(self, robot, name, params=[]):
        self.robot = robot
        self.name = name
        self.params = params

    def execute(self):
        pass 

# Action subclass to define a specific action, in this case "move to position"
class MoveToPosAction (Action):

    # static class variable that will be read by the command processor
    NAME = 'MOVE_TO_POS'

    # overridden init that plugs in the action's name
    def __init__(self, robot, params=[]):
        super().__init(robot, MoveToPosAction.NAME, params)

    # all actions need to override execute to do their thing
    # execute() is called on every robot loop until the action is done
    # returns: True if the action is done, False if not done yet
    def execute(self) -> bool:
        return self.robot.drivetrain.moveToPos(
            self.params[0],
            self.params[1],
            self.params[2],
        )

START_Y = 100
START_X_A = 100
COMMUNITY_CLEAR_X = 200
START_ANGLE = 0

PLACE_CONE = [

]

CLEAR_COMMUNITY = [
    MoveToPosAction(START_X_A, START_Y, START_ANGLE)
]