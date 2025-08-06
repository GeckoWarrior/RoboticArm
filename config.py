# config/config.py

from dataclasses import dataclass

@dataclass
class RobotConfig:
    ip: str = "192.168.0.10"
    target_dt: float = 0.05 # seconds (100Hz loop)
    max_linear_speed: float = 0.1  # m/s
    max_rotational_speed: float = 0.5  # rad/s
    acceleration: float = 0.3  # m/s²
    cam_rel_offset: list[float] = (-0.05, -0.105, -0.15)
    default_desired_rel_pos: list[float] = (-0.05, -0.105, -0.15 + 0.25)

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
