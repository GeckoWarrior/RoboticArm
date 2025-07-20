# config/config.py

from dataclasses import dataclass

@dataclass
class RobotConfig:
    ip: str = "192.168.0.10"
    target_dt: float = 0.01  # seconds (100Hz loop)
    max_linear_speed: float = 0.05  # m/s
    max_rotational_speed: float = 0.2  # rad/s
    acceleration: float = 0.3  # m/s²


@dataclass
class VisionConfig:
    dummy: str = "dummy"


class Config:
    def __init__(self):
        self.robot = RobotConfig()
        # You can later add:
        # self.camera = CameraConfig()
        # self.motion = MotionParams()
