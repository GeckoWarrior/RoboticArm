# config/config.py

from dataclasses import dataclass

@dataclass
class RobotConfig:
    ip: str = "192.168.0.10"
    target_dt: float = 0.01  # seconds (100Hz loop)
    max_linear_speed: float = 0.1  # m/s
    max_rotational_speed: float = 0.2  # rad/s
    acceleration: float = 0.3  # m/s²

@dataclass
class VisionConfig:
    dummy: str = "dummy"

@dataclass
class SystemConfig:
    mode: str = "simulation"  # "live" or "simulation"

class Config:
    def __init__(self):
        self.system = SystemConfig()
        self.robot = RobotConfig()
        self.vision = VisionConfig()
