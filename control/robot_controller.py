from control.robot_interface import RoboticArm
from control.motion_model import SmoothMotionModel, PositionalMotionModel
from config import Config
import numpy as np

class RobotController:
    def __init__(self, robot: RoboticArm, config: Config):
        self.robot = robot
        self.target_camf_pos = None  # 3D target position
        self.desired_pos = np.array(config.robot.default_desired_rel_pos)  # 3D target position
        self.motion_model = PositionalMotionModel(config)
        self.config = config
        self.gain = 0.2

    def set_target_camf_pos(self, pos):
        self.target_camf_pos = pos
    
    def set_desired_toolf_pos(self, pos):
        self.desired_pos = pos

    def update(self, dt, tcp_pose):
        if self.target_camf_pos is None:
            return
        
        pos_toolf_pos = self.target_camf_pos - np.array(self.config.robot.cam_rel_offset)

        vel = self.motion_model.compute_velocity(tcp_pose, pos_toolf_pos, self.desired_pos, dt)
        vel -= self.gain * self.robot.get_tcp_velocity()
        self.robot.move_velocity(vel, acceleration=0.3, dt=dt)
