import numpy as np
from utils.transforms import get_relative_pose

class MovingTarget:
    def __init__(self, robot):
        self.t = 0.0
        self.robot = robot

    def get_target_rel_pose(self, dt):
        self.t += dt
        # Example: a circular trajectory in XY plane
        radius = 0.2
        omega = 0.5  # rad/s
        x = radius * np.cos(omega * self.t)
        y = radius * np.sin(omega * self.t)
        z = 0.3
        
        r_pose = np.array(self.robot.get_tcp_pose())
        t_pose = np.array([x, y, z, 0, 0, 0])

        # relative pose to the robot:
        rel_pose = get_relative_pose(r_pose, t_pose)
        return rel_pose  # pose
