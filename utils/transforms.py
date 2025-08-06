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


def cart_to_homo(vec):
    return np.array([vec[0], vec[1], vec[2], 1])


def homo_to_cart(vec):
    return (vec / vec[3])[:3]
