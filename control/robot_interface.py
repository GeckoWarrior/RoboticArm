from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive


class RoboticArm:
    def __init__(self, ip):
        self.ip = ip
        self.rtde_c = None
        self.rtde_r = None
        self.ready = False

    def setup(self):
        try:
            self.rtde_c = RTDEControl(self.ip)
            self.rtde_r = RTDEReceive(self.ip)
            self.ready = True
            print(f"[INFO] RTDE connection established with {self.ip}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to connect to robot at {self.ip}: {e}")
            self.ready = False
            return False

    def get_tcp_pose(self):
        if not self.ready:
            raise RuntimeError("RTDE not initialized. Call setup() first.")
        try:
            return self.rtde_r.getActualTCPPose()
        except Exception as e:
            print(f"[ERROR] Failed to get TCP pose: {e}")
            return None

    def move_velocity(self, vel, acceleration, dt):
        if not self.ready:
            raise RuntimeError("RTDE not initialized. Call setup() first.")
        try:
            self.rtde_c.speedL(vel, acceleration, dt)
        except Exception as e:
            print(f"[ERROR] Failed to send velocity command: {e}")

    def stop(self):
        if not self.ready:
            return
        try:
            self.rtde_c.speedStop()
            self.rtde_c.stopScript()
            print("[INFO] Robot stopped successfully")
        except Exception as e:
            print(f"[ERROR] Failed to stop robot: {e}")
