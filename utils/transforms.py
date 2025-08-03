import numpy as np
from scipy.spatial.transform import Rotation as R

def pose_to_matrix(pose):
    """Convert 6D pose [x, y, z, rx, ry, rz] to a 4x4 transformation matrix."""
    position = pose[:3]
    rotvec = pose[3:]
    rotation = R.from_rotvec(rotvec).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = position
    return T

def matrix_to_pose(T):
    """Convert 4x4 transformation matrix to 6D pose [x, y, z, rx, ry, rz]."""
    position = T[:3, 3]
    rotation = R.from_matrix(T[:3, :3]).as_rotvec()
    return np.concatenate([position, rotation])

def get_relative_pose(from_pose, to_pose):
    """
    Compute relative pose (to_pose relative to from_pose).
    Both poses are 6D: [x, y, z, rx, ry, rz] (rotation as rotvec).
    """
    T_from = pose_to_matrix(from_pose)
    T_to = pose_to_matrix(to_pose)
    T_rel = np.linalg.inv(T_from) @ T_to
    return matrix_to_pose(T_rel).tolist()

def apply_relative_pose(abs_pose, rel_pose):
    """
    Compute the absolute pose resulting from applying rel_pose on top of abs_pose.
    
    Both poses are 6D: [x, y, z, rx, ry, rz] (rotation as rotvec).
    """
    T_abs = pose_to_matrix(abs_pose)
    T_rel = pose_to_matrix(rel_pose)
    T_result = T_abs @ T_rel
    return matrix_to_pose(T_result).tolist()
