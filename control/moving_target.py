import numpy as np
from utils.transforms import get_relative_pose

class MovingTarget:
    def __init__(self, robot):
        self.t = 0.0
        self.robot = robot
        temp = robot.get_tcp_pose()
        self.x_base = temp[0]
        self.y_base = temp[1]
        self.z_base = temp[2]

    def get_target_rel_pose(self, dt):
        self.t += dt
        # Example: a circular trajectory in XY plane
        radius = 0.5
        omega = 0.7 # rad/s
        x = radius * np.cos(omega * self.t) + self.x_base
        y = radius * np.sin(omega * self.t) + self.y_base
        z = self.z_base
        
        r_pose = np.array(self.robot.get_tcp_pose())
        t_pose = np.array([x, y, z, 0, -3, 0])
        self.pose = t_pose
        # relative pose to the robot:
        rel_pose = get_relative_pose(r_pose, t_pose)
        print(rel_pose)
        print(t_pose - r_pose)
        return rel_pose#.tolist()  # pose
