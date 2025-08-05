from typing import List
from scipy.spatial.transform import Rotation as R
from config import RobotConfig
from utils.transforms import pose_to_matrix, matrix_to_pose, apply_relative_pose
import numpy as np
import math



class PositionalMotionModel:
    def __init__(self, config: RobotConfig):
        self.max_lin = config.robot.max_linear_speed

    def compute_velocity(
        self,
        tcp_base_pose: List[float],
        target_relative_pose: List[float],
        desired_relative_pose: List[float],
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
        assert len(tcp_base_pose) == 6
        assert len(target_relative_pose) == 6
        assert len(desired_relative_pose) == 6

        # Step 1: Compute the position error
        target_pos = target_relative_pose[:3]
        desired_pos = desired_relative_pose[:3]
        position_error = [target_pos[i] - desired_pos[i]  for i in range(3)]

        # Step 2: Compute raw linear velocity
        linear_velocity = [error / dt for error in position_error]

        # Step 3: Clamp linear velocity to max_lin
        norm = math.sqrt(sum(v ** 2 for v in linear_velocity))
        if norm > self.max_lin and norm > 1e-8:
            scale = self.max_lin / norm
            linear_velocity = [v * scale for v in linear_velocity]

        # Step 4: Append zero rotation velocities
        return apply_relative_pose(tcp_base_pose, linear_velocity + [0, 0, 0])




class SmoothMotionModel:
    def __init__(self, config: RobotConfig):
        self.max_lin = config.robot.max_linear_speed
        self.max_rot = config.robot.max_rotational_speed
        self.gain = 1.0  # Proportional control gain

    def compute_velocity(
        self,
        tcp_base_pose: List[float],
        target_relative_pose: List[float],
        desired_relative_pose: List[float],
        dt: float
    ) -> List[float]:
        """
        Computes the velocity vector needed to move the end-effector
        such that the target appears at the desired relative position.
        Inputs and outputs are in 6D (3 pos, 3 rot).
        
        :param tcp_base_pose: [dx, dy, dz, rx, ry, rz] camera in base (axis-angle)
        :param target_relative_pose: [dx, dy, dz, rx, ry, rz] from camera (axis-angle)
        :param desired_relative_pose: [dx, dy, dz, rx, ry, rz] from camera (axis-angle)
        :param dt: time delta in seconds
        :return: velocity vector [vx, vy, vz, wx, wy, wz]
        """
        assert len(tcp_base_pose) == 6
        assert len(target_relative_pose) == 6
        assert len(desired_relative_pose) == 6

        """
        HOW
        1: 
        figure out where the tool should be in tool space
            - calc the error from target and desired
            - respect it to the tcp (the inv trasformation of that)

        2:
        figure out where the tool should be in base space
            - transform the error vec from 1
        
        3:
        construct a velocity vector
            - translation is lerp?
            - rotation is slerp?
            - capped to the max velocities
        """

        # Convert poses to matrices
        T_tcp_base = pose_to_matrix(tcp_base_pose)
        T_target_tool = pose_to_matrix(target_relative_pose)
        T_desired_tool = pose_to_matrix(desired_relative_pose)

        # Calculate the error in the tool frame (desired^-1 * target)
        T_err_tool = np.linalg.inv(T_desired_tool) @ T_target_tool

        # Transform the error into the base frame
        T_err_base = T_tcp_base @ T_err_tool @ np.linalg.inv(T_tcp_base)

        # Convert error transformation to velocity twist
        twist = matrix_to_pose(T_err_base)

        # Scale by dt to get velocity
        velocity = twist / dt

        # Clamp linear velocity
        for i in range(3):
            velocity[i] = max(min(velocity[i], self.max_lin), -self.max_lin)
        for i in range(3):
            velocity[i + 3] = max(min(velocity[i + 3], self.max_rot), -self.max_rot)

        return velocity.tolist()



        # --- Linear velocity control (same as before) ---
        lin_error = [
            (t - d) / dt for t, d in zip(target_relative_pose[:3], desired_relative_pose[:3])
        ]

        # Clamp linear velocity
        for i in range(3):
            lin_error[i] = max(min(lin_error[i], self.max_lin), -self.max_lin)

        # --- Angular velocity control (new, using quaternions) ---
        rot_error_vec = self._rotation_vector_error(
            current_rvec=target_relative_pose[3:],
            desired_rvec=desired_relative_pose[3:]
        )

        # Proportional angular velocity
        rot_velocity = [self.gain * r / dt for r in rot_error_vec]

        # Clamp angular velocity
        for i in range(3):
            rot_velocity[i] = max(min(rot_velocity[i], self.max_rot), -self.max_rot)

        return lin_error + rot_velocity

    def _rotation_vector_error(self, current_rvec, desired_rvec):
        """
        Computes the angular error vector (in axis-angle form) between
        current and desired rotation vectors.
        """
        r_current = R.from_rotvec(current_rvec)
        r_desired = R.from_rotvec(desired_rvec)

        # Relative rotation (what rotation to apply to current to reach desired)
        r_error = r_desired * r_current.inv()

        return r_error.as_rotvec()
