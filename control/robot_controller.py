from control.robot_interface import RoboticArm
from control.motion_model import SmoothMotionModel
from config import Config

class RobotController:
    def __init__(self, robot: RoboticArm, config: Config):
        self.robot = robot
        self.target_pose = None  # 6D target position
        self.desired_pose = None  # 6D target position
        self.motion_model = SmoothMotionModel(config)

    def set_target_pose(self, pose6d):
        self.target_pose = pose6d
    
    def set_desired_pose(self, pose6d):
        self.desired_pose = pose6d

    def update(self, dt):
        if not self.target_pose:
            return

        vel = self.motion_model.compute_velocity(self.target_pose, self.desired_pose, dt)
        self.robot.move_velocity(vel, acceleration=0.3, dt=dt)
