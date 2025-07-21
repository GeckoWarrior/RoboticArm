from typing import List
from scipy.spatial.transform import Rotation as R
from config import RobotConfig

class SmoothMotionModel:
    def __init__(self, config: RobotConfig):
        self.max_lin = config.robot.max_linear_speed
        self.max_rot = config.robot.max_rotational_speed
        self.gain = 1.0  # Proportional control gain

    def compute_velocity(
        self,
        target_relative_pose: List[float],
        desired_relative_pose: List[float],
        dt: float
    ) -> List[float]:
        """
        Computes the velocity vector needed to move the end-effector
        such that the target appears at the desired relative position.
        Inputs and outputs are in 6D (3 pos, 3 rot).
        
        :param target_relative_pose: [dx, dy, dz, rx, ry, rz] from camera (axis-angle)
        :param desired_relative_pose: [dx, dy, dz, rx, ry, rz] (axis-angle)
        :param dt: time delta in seconds
        :return: velocity vector [vx, vy, vz, wx, wy, wz]
        """
        assert len(target_relative_pose) == 6
        assert len(desired_relative_pose) == 6

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
