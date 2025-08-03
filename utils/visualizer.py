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

        self.robot_point = None
        self.target_point = None
        self.robot_axes = []
        self.target_axes = []

        self._setup_plot()
        plt.ion()
        plt.show()

    def _setup_plot(self):
        self.ax.set_xlim(*self.xlim)
        self.ax.set_ylim(*self.ylim)
        self.ax.set_zlim(*self.zlim)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Robot and Target Visualization')
        self.ax.legend(['Robot', 'Target'])

    def _pose_to_matrix(self, pose):
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        if len(pose) == 6:
            T[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
        return T

    def _draw_axes(self, T, length=0.05, alpha=0.8):
        origin = T[:3, 3]
        rot = T[:3, :3]
        axes = []

        x_axis = rot @ np.array([length, 0, 0])
        y_axis = rot @ np.array([0, length, 0])
        z_axis = rot @ np.array([0, 0, length])

        axes.append(self.ax.quiver(*origin, *x_axis, color='r', alpha=alpha))
        axes.append(self.ax.quiver(*origin, *y_axis, color='g', alpha=alpha))
        axes.append(self.ax.quiver(*origin, *z_axis, color='b', alpha=alpha))
        return axes

    def update(self, robot_pose, target_pose):
        # Remove old plot elements
        if self.robot_point:
            self.robot_point.remove()
        if self.target_point:
            self.target_point.remove()
        for arrow in self.robot_axes + self.target_axes:
            arrow.remove()

        # Generate new transforms
        T_robot = self._pose_to_matrix(robot_pose)
        T_target = self._pose_to_matrix(target_pose)

        # Plot new points
        self.robot_point = self.ax.scatter(*T_robot[:3, 3], c='b')
        self.target_point = self.ax.scatter(*T_target[:3, 3], c='r')

        # Draw axes and store references
        self.robot_axes = self._draw_axes(T_robot, length=0.05, alpha=0.9)
        self.target_axes = self._draw_axes(T_target, length=0.05, alpha=0.5)

        # Efficient canvas update
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
