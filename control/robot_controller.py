from robot_interface import RoboticArm
from motion_model import SmoothMotionModel

class RobotController:
    def __init__(self, robot: RoboticArm):
        self.robot = robot
        self.target_pose = None  # 6D target position
        self.motion_model = SmoothMotionModel()

    def set_target_pose(self, pose6d):
        self.target_pose = pose6d

    def update(self, dt):
        if not self.target_pose:
            return

        current_pose = self.robot.get_tcp_pose()
        vel = self.motion_model.compute_velocity(current_pose, self.target_pose, dt)
        self.robot.move_velocity(vel, acceleration=0.3, dt=dt)
