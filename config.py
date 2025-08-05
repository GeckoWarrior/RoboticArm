# config/config.py

from dataclasses import dataclass, field

@dataclass
class RobotConfig:
    ip: str = "192.168.0.10"
    target_dt: float = 0.02  # seconds (100Hz loop)
    max_linear_speed: float = 0.4  # m/s
    max_rotational_speed: float = 0.05  # rad/s
    acceleration: float = 0.3  # m/s²
    default_desired_rel_pose: list[float] = (0, 0, 0, 0, 0, 0)  # 6D relative pose

@dataclass
class VisionConfig:
    cam: str = "robot" # "laptop" or "robot"
    depth: int = 0.5
    K:  list = field(default_factory=lambda: [[598.85657176, 0, 319.35979167], [0, 598.52225975, 244.8183713], [0, 0, 1]])

@dataclass
class SystemConfig:
    mode: str = "simulation"  # "live" or "simulation"

class Config:
    def __init__(self):
        self.system = SystemConfig()
        self.robot = RobotConfig()
        self.vision = VisionConfig()
