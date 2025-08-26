from typing import List
from scipy.spatial.transform import Rotation as R
from config import RobotConfig
from utils.transforms import *
import numpy as np
import math


class PositionalMotionModel:
    def __init__(self, config: RobotConfig):
        self.max_lin = config.robot.max_linear_speed
        self.close_distance = config.robot.close_distance

    def compute_velocity(
        self,
        pose_basef_tool: np.ndarray,
        pos_toolf_target: np.ndarray,
        pos_toolf_desired_target: np.ndarray,
        dt: float
    ) -> List[float]:
        """
        Computes the velocity vector needed to move the end-effector
        such that the target appears at the desired relative position.
        Inputs and outputs are in 6D (3 pos, 3 rot).

        The PositionalMotionModel igonres the rotation values, and only sets the postion correctly!
        
        :param tcp_base_pose: [dx, dy, dz, rx, ry, rz] camera in base (axis-angle)
        :param target_relative_pose: [dx, dy, dz] from camera (axis-angle)
        :param desired_relative_pose: [dx, dy, dz] from camera (axis-angle)
        :param dt: time delta in seconds
        :return: velocity vector [vx, vy, vz, wx=0, wy=0, wz=0] (rotations are ignored)
        """
        assert len(pose_basef_tool) == 6
        assert len(pos_toolf_target) == 3
        assert len(pos_toolf_desired_target) == 3

        # Frame transformations & Velocity calculation
        pos_toolf_target = cart_to_homo(pos_toolf_target)
        pos_toolf_desired_target = cart_to_homo(pos_toolf_desired_target)

        T_basef_tool = pose_to_matrix(pose_basef_tool)

        pos_basef_target = T_basef_tool @ pos_toolf_target 
        pos_basef_desired_target = T_basef_tool @ pos_toolf_desired_target 
    
        delta_pos = homo_to_cart(pos_basef_target) - homo_to_cart(pos_basef_desired_target)
    
        v_linear = delta_pos / dt

        # Slowdown mechanism
        dist = np.linalg.norm(delta_pos)
        max_lin = min(self.max_lin, self.max_lin * (dist / self.close_distance))
        if dist < 0.01:
            max_lin = 0

        # Locking / Jiggle prevention
        norm = math.sqrt(sum(v ** 2 for v in v_linear))
        if norm > max_lin and norm > 1e-8:
            scale = max_lin / norm
            v_linear *= scale

        # Format as command
        return v_linear.tolist() + [0,0,0]
    

class RotationalMotionModel:
    def __init__(self, config: RobotConfig):
        self.max_lin = config.robot.max_linear_speed
        self.max_ang = config.robot.max_rotational_speed

    def compute_velocity(
        self,
        pose_basef_tool: List[float],
        pose_toolf_target: List[float],
        pose_toolf_desired_target: List[float],
        dt: float
    ) -> List[float]:
        """
        Computes the velocity vector needed to move the end-effector
        such that the target appears at the desired relative position.
        Inputs and outputs are in 6D (3 pos, 3 rot).

        The PositionalMotionModel igonres the rotation values, and only sets the postion correctly!
        
        :param tcp_base_pose: [dx, dy, dz, rx, ry, rz] camera in base (axis-angle)
        :param target_relative_pose: [dx, dy, dz, rx, ry, rz] from camera (axis-angle)
        :param desired_relative_pose: [dx, dy, dz, rx, ry, rz] from camera (axis-angle)
        :param dt: time delta in seconds
        :return: velocity vector [vx, vy, vz, wx=0, wy=0, wz=0] (rotations are ignored)
        """
        assert len(pose_basef_tool) == 6
        assert len(pose_toolf_target) == 6
        assert len(pose_toolf_desired_target) == 6

        # 1. Get current TCP pose
        T_basef_tool = pose_to_matrix(pose_basef_tool)

        # 3. Convert to base frame
        T_toolf_target = pose_to_matrix(pose_toolf_target)
        T_basef_target = T_basef_tool @ T_toolf_target

        # 4. Desired position of target in tool frame
        temp = np.array(pose_toolf_desired_target) 
        temp[:3] = np.array(pose_toolf_target)[:3]
        pose_toolf_desired_target = temp.tolist()
        T_toolf_desired_target = pose_to_matrix(pose_toolf_desired_target)

        # 5. Compute desired tool pose
        T_basef_tool_new = T_basef_target @ np.linalg.inv(T_toolf_desired_target)

        # 6. Compute velocity
        dt = 0.05
        delta_pos = T_basef_tool_new[:3, 3] - T_basef_tool[:3, 3]
        R_current = T_basef_tool[:3, :3]
        R_desired = T_basef_tool_new[:3, :3]
        R_delta = R_desired @ R_current.T
        rotvec = R.from_matrix(R_delta).as_rotvec()

        v_linear = delta_pos / dt
        v_angular = rotvec / dt

        # Optional: clamp to max speed
        v_linear = np.clip(v_linear, -self.max_lin, self.max_lin)
        v_angular = np.clip(v_angular, -self.max_ang, self.max_ang)

        # 7. Send command
        return list(v_linear) + list(v_angular)



class SmoothMotionModel:
    def __init__(self, config: RobotConfig):
        self.max_lin = config.robot.max_linear_speed
        self.max_ang = config.robot.max_rotational_speed
        self.gain = 1.0  # Proportional control gain

    def compute_velocity(
        self,
        pose_basef_tool: List[float],
        pose_toolf_target: List[float],
        pose_toolf_desired_target: List[float],
        dt: float
    ) -> List[float]:
        """
        Computes the velocity vector needed to move the end-effector
        such that the target appears at the desired relative position.

        :param pose_basef_tool: [x, y, z, rx, ry, rz] (TCP/tool in base frame)
        :param pose_toolf_target: [x, y, z, rx, ry, rz] (target relative to tool)
        :param pose_toolf_desired_target: [x, y, z, rx, ry, rz] (desired target position in tool frame)
        :param dt: time delta
        :return: velocity vector [vx, vy, vz, wx, wy, wz]
        """
        assert len(pose_basef_tool) == 6
        assert len(pose_toolf_target) == 6
        assert len(pose_toolf_desired_target) == 6

        # 1. Convert poses to 4x4 matrices
        T_basef_tool = pose_to_matrix(np.array(pose_basef_tool))
        T_toolf_target = pose_to_matrix(np.array(pose_toolf_target))
        T_toolf_desired_target = pose_to_matrix(np.array(pose_toolf_desired_target))

        # 2. Compute target pose in base frame
        T_basef_target = T_basef_tool @ T_toolf_target

        # 3. Compute desired tool pose in base frame
        T_basef_tool_new = T_basef_target @ np.linalg.inv(T_toolf_desired_target)

        # 4. Linear velocity
        pos_current = T_basef_tool[:3, 3]
        pos_desired = T_basef_tool_new[:3, 3]
        v_linear = (pos_desired - pos_current) / dt

        # 5. Angular velocity using quaternions
        R_current = R.from_matrix(T_basef_tool[:3, :3])
        R_desired = R.from_matrix(T_basef_tool_new[:3, :3])

        # Compute rotation difference
        R_delta = R_desired * R_current.inv()
        rotvec = R_delta.as_rotvec()
        v_angular = rotvec / dt

        # 6. Clamp if needed
        v_linear = np.clip(v_linear, -self.max_lin, self.max_lin)
        v_angular = np.clip(v_angular, -self.max_ang, self.max_ang)

        return list(v_linear) + list(v_angular)

    