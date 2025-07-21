# utils/visualizer.py
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Visualizer:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()
        plt.show()

    def update(self, robot_pose, target_pose):
        self.ax.clear()
        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_zlim(0, 0.6)
        self.ax.scatter(*robot_pose[:3], c='b', label='Robot')
        self.ax.scatter(*target_pose[:3], c='r', label='Target')
        self.ax.legend()
        plt.draw()
        plt.pause(0.001) # not good!
