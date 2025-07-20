from config import Config
from control.robot_interface import RoboticArm
from control.robot_controller import RobotController
from utils.timing import Clock

config = Config()

# Setup robot interface
robot = RoboticArm(config.robot.ip)
controller = RobotController(robot)

# Setup timer
timer = Clock()

try:
    while True:
        dt = timer.tick()
        controller.update(dt)
except KeyboardInterrupt:
    robot.stop()
