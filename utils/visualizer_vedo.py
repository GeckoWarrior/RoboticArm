from vedo import Plotter, Sphere, Arrow
import numpy as np
from scipy.spatial.transform import Rotation as R

class Visualizer:
    def __init__(self):
        self.plotter = Plotter(interactive=False, axes=1, title="Robot and Target Visualization")
        self.robot_point = None
        self.target_point = None
        self.robot_axes = []
        self.target_axes = []

        self.plotter.show(resetcam=True, interactive=False)  # Show once, don't block
        self.plotter.interactor.Initialize()  # Critical for making the window interactive

    def _pose_to_matrix(self, pose):
        T = np.eye(4)
        T[:3, 3] = pose[:3]
        if len(pose) == 6:
            T[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
        return T

    def _draw_axes(self, T, length=0.05, alpha=0.8):
        origin = T[:3, 3]
        rot = T[:3, :3]

        x_axis = origin + rot @ np.array([length, 0, 0])
        y_axis = origin + rot @ np.array([0, length, 0])
        z_axis = origin + rot @ np.array([0, 0, length])

        return [
            Arrow(origin, x_axis, c='r', alpha=alpha),
            Arrow(origin, y_axis, c='g', alpha=alpha),
            Arrow(origin, z_axis, c='b', alpha=alpha),
        ]

    def update(self, robot_pose, target_pose):
        for obj in [self.robot_point, self.target_point, *self.robot_axes, *self.target_axes]:
            if obj is not None:
                self.plotter.remove(obj)

        T_robot = self._pose_to_matrix(robot_pose)
        T_target = self._pose_to_matrix(target_pose)

        self.robot_point = Sphere(pos=T_robot[:3, 3], r=0.01, c='blue')
        self.target_point = Sphere(pos=T_target[:3, 3], r=0.01, c='red')
        self.robot_axes = self._draw_axes(T_robot, 0.05, alpha=0.9)
        self.target_axes = self._draw_axes(T_target, 0.05, alpha=0.5)

        self.plotter.add(self.robot_point, self.target_point, *self.robot_axes, *self.target_axes)
        self.plotter.render()

    def close(self):
        self.plotter.close()
