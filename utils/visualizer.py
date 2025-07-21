# utils/visualizer.py
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

class Visualizer:
    def __init__(self, xlim=(-0.5, 0.5), ylim=(-0.5, 0.5), zlim=(0, 0.6)):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.xlim = xlim
        self.ylim = ylim
        self.zlim = zlim

        plt.ion()
        plt.show()

    def _pose_to_matrix(self, pose):
        """
        Convert 6D pose [x, y, z, rx, ry, rz] to 4x4 transformation matrix.
        Rotation is in axis-angle format (Rodrigues).
        """
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        if len(pose) == 6:
            T[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
        return T

    def _draw_axes(self, T, length=0.05, alpha=0.8):
        """
        Draw coordinate frame axes at transformation T.
        """
        origin = T[:3, 3]
        rot = T[:3, :3]

        x_axis = rot @ np.array([length, 0, 0])
        y_axis = rot @ np.array([0, length, 0])
        z_axis = rot @ np.array([0, 0, length])

        self.ax.quiver(*origin, *x_axis, color='r', alpha=alpha)
        self.ax.quiver(*origin, *y_axis, color='g', alpha=alpha)
        self.ax.quiver(*origin, *z_axis, color='b', alpha=alpha)

    def set_limits(self, xlim=None, ylim=None, zlim=None):
        if xlim:
            self.xlim = xlim
        if ylim:
            self.ylim = ylim
        if zlim:
            self.zlim = zlim

    def update(self, robot_pose, target_pose):
        """
        Accepts robot_pose and target_pose as 6D vectors (x, y, z, rx, ry, rz).
        """
        T_robot = self._pose_to_matrix(robot_pose)
        T_target = self._pose_to_matrix(target_pose)

        self.ax.clear()
        self.ax.set_xlim(*self.xlim)
        self.ax.set_ylim(*self.ylim)
        self.ax.set_zlim(*self.zlim)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # Plot robot and target points
        self.ax.scatter(*T_robot[:3, 3], c='b', label='Robot')
        self.ax.scatter(*T_target[:3, 3], c='r', label='Target')

        # Draw axes
        self._draw_axes(T_robot, length=0.05, alpha=0.9)
        self._draw_axes(T_target, length=0.05, alpha=0.5)

        self.ax.legend()
        plt.draw()
        plt.pause(0.001) # TODO: fix this later