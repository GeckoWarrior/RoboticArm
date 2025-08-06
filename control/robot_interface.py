from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import math
import time
import numpy as np

class RoboticArm:
    def __init__(self, config):
        self.ip = config.robot.ip
        self.freq = int(1/config.robot.target_dt)
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
    
    def go_home(self):
        # --- Target joint configuration (in radians) ---
        target_joints = [
            math.radians(0),      # Base
            math.radians(-90),    # Shoulder
            math.radians(-90),    # Elbow
            math.radians(270),    # Wrist1
            math.radians(90),     # Wrist2
            math.radians(-270)    # Wrist3
        ]

        speed = 1.0       # rad/s
        accel = 0.5       # rad/s^2

        print("Moving to target joint configuration...")
        self.rtde_c.moveJ(target_joints, speed, accel)

        # --- Wait until robot stops moving ---
        while any(abs(v) > 0.001 for v in self.rtde_r.getActualTCPSpeed()):
            time.sleep(0.05)
        print("Reached target joint position.")

    def get_tcp_pose(self):
        if not self.ready:
            raise RuntimeError("RTDE not initialized. Call setup() first.")
        try:
            return np.array(self.rtde_r.getActualTCPPose())
        except Exception as e:
            print(f"[ERROR] Failed to get TCP pose: {e}")
            return None
        
    def get_tcp_velocity(self):
        if not self.ready:
            raise RuntimeError("RTDE not initialized. Call setup() first.")
        try:
            return np.array(self.rtde_r.getActualTCPSpeed())
        except Exception as e:
            print(f"[ERROR] Failed to get TCP velocity: {e}")
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
    
    def disconnect(self):
        self.rtde_c.disconnect()
        self.rtde_r.disconnect()
