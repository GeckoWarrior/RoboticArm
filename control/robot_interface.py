from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

class RoboticArm:
    def __init__(self, ip):
        self.rtde_c = RTDEControl(ip)
        self.rtde_r = RTDEReceive(ip)
        self.running = True

    def get_tcp_pose(self):
        return self.rtde_r.getActualTCPPose()

    def move_velocity(self, vel, acceleration, dt):
        self.rtde_c.speedL(vel, acceleration, dt)

    def stop(self):
        self.rtde_c.speedStop()
        self.rtde_c.stopScript()
        self.running = False
