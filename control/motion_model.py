from typing import List
from scipy.spatial.transform import Rotation as R
from config import RobotConfig
from utils.transforms import *
import numpy as np
import math


class PositionalMotionModel:
    def __init__(self, config: RobotConfig):
        self.max_lin = config.robot.max_linear_speed

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
    
        pos_toolf_target = cart_to_homo(pos_toolf_target)
        pos_toolf_desired_target = cart_to_homo(pos_toolf_desired_target)

        # 1. Get current TCP pose
        T_basef_tool = pose_to_matrix(pose_basef_tool)

        pos_basef_target = T_basef_tool @ pos_toolf_target 
        pos_basef_desired_target = T_basef_tool @ pos_toolf_desired_target 
    
        delta_pos = homo_to_cart(pos_basef_target) - homo_to_cart(pos_basef_desired_target)

        v_linear = delta_pos / dt

        # Optional: clamp to max speed
        norm = math.sqrt(sum(v ** 2 for v in v_linear))
        if norm > self.max_lin and norm > 1e-8:
            scale = self.max_lin / norm
            v_linear *= scale

        # 7. Send command
        return list(v_linear) + [0,0,0]
    
# class PositionalMotionModel:
#     def __init__(self, config: RobotConfig):
#         self.max_lin = config.robot.max_linear_speed
#         self.max_ang = config.robot.max_rotational_speed

#     def compute_velocity(
#         self,
#         pose_basef_tool: List[float],
#         pose_toolf_target: List[float],
#         pose_toolf_desired_target: List[float],
#         dt: float
#     ) -> List[float]:
#         """
#         Computes the velocity vector needed to move the end-effector
#         such that the target appears at the desired relative position.
#         Inputs and outputs are in 6D (3 pos, 3 rot).

#         The PositionalMotionModel igonres the rotation values, and only sets the postion correctly!
        
#         :param tcp_base_pose: [dx, dy, dz, rx, ry, rz] camera in base (axis-angle)
#         :param target_relative_pose: [dx, dy, dz, rx, ry, rz] from camera (axis-angle)
#         :param desired_relative_pose: [dx, dy, dz, rx, ry, rz] from camera (axis-angle)
#         :param dt: time delta in seconds
#         :return: velocity vector [vx, vy, vz, wx=0, wy=0, wz=0] (rotations are ignored)
#         """
#         assert len(pose_basef_tool) == 6
#         assert len(pose_toolf_target) == 6
#         assert len(pose_toolf_desired_target) == 6
    
#         pos_toolf_target = np.arange(3)
#         pos_toolf_desired_target = np.arange(3)

#         vec_toolf_delta = pos_toolf_target - pos_toolf_desired_target


#         # 1. Get current TCP pose
#         T_basef_tool = pose_to_matrix(pose_basef_tool)

#         # 3. Convert to base frame
#         pose_toolf_target[3] = 0
#         pose_toolf_target[4] = 0
#         pose_toolf_target[5] = 0
#         T_toolf_target = pose_to_matrix(pose_toolf_target)
#         T_basef_target = T_basef_tool @ T_toolf_target

#         # 4. Desired position of target in tool frame
#         temp = np.array(pose_toolf_desired_target) 
#         temp[3:] = np.array(pose_toolf_target)[3:]
#         pose_toolf_desired_target = temp.tolist()
#         T_toolf_desired_target = pose_to_matrix(pose_toolf_desired_target)

#         # 5. Compute desired tool pose
#         T_basef_tool_new = T_basef_target @ np.linalg.inv(T_toolf_desired_target)

#         # 6. Compute velocity
#         delta_pos = T_basef_tool_new[:3, 3] - T_basef_tool[:3, 3]
#         R_current = T_basef_tool[:3, :3]
#         R_desired = T_basef_tool_new[:3, :3]
#         R_delta = R_desired @ R_current.T
#         rotvec = R.from_matrix(R_delta).as_rotvec()

#         v_linear = delta_pos / dt
#         v_angular = rotvec / dt

#         # Optional: clamp to max speed
#         v_linear = np.clip(v_linear, -self.max_lin, self.max_lin)
#         v_angular = np.clip(v_angular, -self.max_ang, self.max_ang)

#         # 7. Send command
#         return list(v_linear) + list(v_angular)


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
        return [0,0,0] + list(v_angular) #list(v_linear) + [0,0,0]#+ list(v_angular)


        # # Step 1: Compute the position error
        # target_pos = target_relative_pose[:3]
        # desired_pos = desired_relative_pose[:3]
        # position_error = [target_pos[i] - desired_pos[i]  for i in range(3)]

        # # Step 2: Compute raw linear velocity
        # linear_velocity = [error / dt for error in position_error]

        # # Step 3: Clamp linear velocity to max_lin
        # norm = math.sqrt(sum(v ** 2 for v in linear_velocity))
        # if norm > self.max_lin and norm > 1e-8:
        #     scale = self.max_lin / norm
        #     linear_velocity = [v * scale for v in linear_velocity]

        # # Step 4: Append zero rotation velocities
        # return apply_relative_pose(tcp_base_pose, linear_velocity + [0, 0, 0])





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

    

# class SmoothMotionModel:
#     def __init__(self, config: RobotConfig):
#         self.max_lin = config.robot.max_linear_speed
#         self.max_rot = config.robot.max_rotational_speed
#         self.gain = 1.0  # Proportional control gain

#     def compute_velocity(
#         self,
#         tcp_base_pose: List[float],
#         target_relative_pose: List[float],
#         desired_relative_pose: List[float],
#         dt: float
#     ) -> List[float]:
#         """
#         Computes the velocity vector needed to move the end-effector
#         such that the target appears at the desired relative position.
#         Inputs and outputs are in 6D (3 pos, 3 rot).
        
#         :param tcp_base_pose: [dx, dy, dz, rx, ry, rz] camera in base (axis-angle)
#         :param target_relative_pose: [dx, dy, dz, rx, ry, rz] from camera (axis-angle)
#         :param desired_relative_pose: [dx, dy, dz, rx, ry, rz] from camera (axis-angle)
#         :param dt: time delta in seconds
#         :return: velocity vector [vx, vy, vz, wx, wy, wz]
#         """
#         assert len(tcp_base_pose) == 6
#         assert len(target_relative_pose) == 6
#         assert len(desired_relative_pose) == 6

#         """
#         HOW
#         1: 
#         figure out where the tool should be in tool space
#             - calc the error from target and desired
#             - respect it to the tcp (the inv trasformation of that)

#         2:
#         figure out where the tool should be in base space
#             - transform the error vec from 1
        
#         3:
#         construct a velocity vector
#             - translation is lerp?
#             - rotation is slerp?
#             - capped to the max velocities
#         """

#         # Convert poses to matrices
#         T_tcp_base = pose_to_matrix(tcp_base_pose)
#         T_target_tool = pose_to_matrix(target_relative_pose)
#         T_desired_tool = pose_to_matrix(desired_relative_pose)

#         # Calculate the error in the tool frame (desired^-1 * target)
#         T_err_tool = np.linalg.inv(T_desired_tool) @ T_target_tool

#         # Transform the error into the base frame
#         T_err_base = T_tcp_base @ T_err_tool @ np.linalg.inv(T_tcp_base)

#         # Convert error transformation to velocity twist
#         twist = matrix_to_pose(T_err_base)

#         # Scale by dt to get velocity
#         velocity = twist / dt

#         # Clamp linear velocity
#         for i in range(3):
#             velocity[i] = max(min(velocity[i], self.max_lin), -self.max_lin)
#         for i in range(3):
#             velocity[i + 3] = max(min(velocity[i + 3], self.max_rot), -self.max_rot)

#         return velocity.tolist()



#         # --- Linear velocity control (same as before) ---
#         lin_error = [
#             (t - d) / dt for t, d in zip(target_relative_pose[:3], desired_relative_pose[:3])
#         ]

#         # Clamp linear velocity
#         for i in range(3):
#             lin_error[i] = max(min(lin_error[i], self.max_lin), -self.max_lin)

#         # --- Angular velocity control (new, using quaternions) ---
#         rot_error_vec = self._rotation_vector_error(
#             current_rvec=target_relative_pose[3:],
#             desired_rvec=desired_relative_pose[3:]
#         )

#         # Proportional angular velocity
#         rot_velocity = [self.gain * r / dt for r in rot_error_vec]

#         # Clamp angular velocity
#         for i in range(3):
#             rot_velocity[i] = max(min(rot_velocity[i], self.max_rot), -self.max_rot)

#         return lin_error + rot_velocity

#     def _rotation_vector_error(self, current_rvec, desired_rvec):
#         """
#         Computes the angular error vector (in axis-angle form) between
#         current and desired rotation vectors.
#         """
#         r_current = R.from_rotvec(current_rvec)
#         r_desired = R.from_rotvec(desired_rvec)

#         # Relative rotation (what rotation to apply to current to reach desired)
#         r_error = r_desired * r_current.inv()

#         return r_error.as_rotvec()
