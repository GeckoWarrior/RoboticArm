import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation as R
from utils.transforms import pose_to_matrix

class Visualizer:
    # Line styling constants
    RADIUS = 0.05
    AXIS_LENGTH = 0.4
    AXIS_WIDTH = 3
    AXIS_COLORS = ['red', 'green', 'blue']

    def __init__(self, xlim=(-1, 1), ylim=(-1, 1), zlim=(-1, 0)):
        # pv.set_plot_theme('dark')
        self.plotter = pv.Plotter(window_size=[400, 300])
        self.plotter.set_background('white')
        self.plotter.show_grid(color='lightgray', bounds= list(xlim) + list(ylim) + list(zlim))
        self.plotter.add_axes()

        self.xlim = xlim
        self.ylim = ylim
        self.zlim = zlim

        # Create robot and target spheres once
        self.robot_sphere = pv.Sphere(radius=self.RADIUS)
        self.target_sphere = pv.Sphere(radius=self.RADIUS)
        self.cam_sphere = pv.Cube(x_length=self.RADIUS, y_length=self.RADIUS, z_length=self.RADIUS)

        self.robot_actor = self.plotter.add_mesh(self.robot_sphere, color='blue')
        self.target_actor = self.plotter.add_mesh(self.target_sphere, color='red')
        self.cam_actor = self.plotter.add_mesh(self.target_sphere, color='green')

        # Create axis line actors once
        self.robot_axes_actors = self._create_axes_actors(opacity=1.0)
        self.cam_axes_actors = self._create_axes_actors(opacity=1.0)

        self.plotter.show(auto_close=False, interactive_update=True)

    def _create_line(self, start, end):
        return pv.Line(start, end)

    def _create_axes_actors(self, opacity):
        origin = np.zeros(3)
        end = np.array([self.AXIS_LENGTH, 0, 0])  # dummy direction
        actors = []
        for color in self.AXIS_COLORS:
            line = self._create_line(origin, end)
            actor = self.plotter.add_mesh(line, color=color, opacity=opacity, line_width=self.AXIS_WIDTH)
            actors.append(actor)
        return actors

    def update(self, robot_pose, target_pos, cam_pose):
        T_robot = pose_to_matrix(robot_pose)
        T_cam = pose_to_matrix(cam_pose)

        self.robot_actor.SetPosition(*T_robot[:3, 3])
        self.cam_actor.SetPosition(*T_cam[:3, 3])
        self.target_actor.SetPosition(target_pos)

        axis_unit_vectors = [np.array([1, 0, 0]),
                             np.array([0, 1, 0]),
                             np.array([0, 0, 1])]

        # Update robot axes
        for i, actor in enumerate(self.robot_axes_actors):
            origin = T_robot[:3, 3]
            direction = T_robot[:3, :3] @ axis_unit_vectors[i] * self.AXIS_LENGTH
            end = origin + direction
            line = self._create_line(origin, end)
            actor.GetMapper().SetInputData(line)
            actor.GetMapper().Update()

        # Update cam axes
        for i, actor in enumerate(self.cam_axes_actors):
            origin = T_cam[:3, 3]
            direction = T_cam[:3, :3] @ axis_unit_vectors[i] * self.AXIS_LENGTH
            end = origin + direction
            line = self._create_line(origin, end)
            actor.GetMapper().SetInputData(line)
            actor.GetMapper().Update()

        self.plotter.update()

    def close(self):
        self.plotter.close()
