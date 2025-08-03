# config/config.py

from dataclasses import dataclass

@dataclass
class RobotConfig:
    ip: str = "192.168.0.10"
    target_dt: float = 0.1  # seconds (100Hz loop)
    max_linear_speed: float = 0.1  # m/s
    max_rotational_speed: float = 0.2  # rad/s
    acceleration: float = 0.3  # m/s²
    default_desired_rel_pose: list[float] = (0, 0, 0.5, 0, 0, 0)  # 6D relative pose

@dataclass
class VisionConfig:
    cam: str = "robot" # "laptop" or "robot"

@dataclass
class SystemConfig:
    mode: str = "simulation"  # "live" or "simulation"

class Config:
    def __init__(self):
        self.system = SystemConfig()
        self.robot = RobotConfig()
        self.vision = VisionConfig()
