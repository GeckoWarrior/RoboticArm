import numpy as np

class SimulatedRoboticArm:
    def __init__(self):
        self.pose = np.zeros(6)  # x, y, z, rx, ry, rz
        self.pose[3] = 1
        self.pose[4] = 0
        self.pose[5] = 2
        self.velocity = np.zeros(6)

    def setup(self):
        return True

    def get_tcp_pose(self):
        return self.pose.copy()

    def move_velocity(self, vel, acceleration, dt):
        # Clip velocities for realism (optional)
        vel = np.array(vel)
        self.pose += vel * dt
        self.velocity = vel

    def stop(self):
        self.velocity[:] = 0
