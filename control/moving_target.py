import numpy as np
from utils.transforms import *
from config import Config

class MovingTarget:
    def __init__(self, robot, config: Config):
        self.t = 0.0
        self.robot = robot
        self.config = config
        temp = robot.get_tcp_pose()
        self.x_base = temp[0]
        self.y_base = temp[1]
        self.z_base = temp[2]

    def get_target_camf_pos(self, dt, pose_basef_tool):
        self.t += dt
        # self.t += 0
        # Example: a circular trajectory in XY plane
        radius = 0.2
        omega = 0.4 # rad/s
        x = radius * np.cos(omega * self.t) + self.x_base
        y = self.y_base#radius * np.sin(0.8 * omega * self.t) + self.y_base
        z = 0#0.8 * radius * np.sin(0.5 * omega * self.t) + self.z_base
        z = self.z_base
        
        self.pos = np.array([x, y, z])
        pos_toolf_target = homo_to_cart(np.linalg.inv(pose_to_matrix(pose_basef_tool)) @ cart_to_homo(self.pos))
        pos_camf_target = pos_toolf_target - np.array(self.config.robot.cam_rel_offset)
        return pos_camf_target
