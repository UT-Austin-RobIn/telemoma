#### code adapted from https://github.com/AlexanderKhazatsky/R2D2/blob/main/r2d2/misc/transformations.py ####

import numpy as np
from scipy.spatial.transform import Rotation as R

canonical_quat = True   # if True then w is always +ve

def rot2quat(rot, canonical_quat=False):
    """
    A custom wrapper of scipy's Rotation class to convert a rotation matrix to a quaternion
    """
    quat = R.as_quat(rot)
    if canonical_quat:
        # Ensure the first nonzero value of [w, x, y, z] is positive
        if quat[-1] < 0:
            quat = -quat
        elif quat[-1] == 0:
            if quat[0] < 0:
                quat = -quat
            elif quat[0] == 0:
                if quat[1] < 0:
                    quat = -quat
                elif quat[1] == 0:
                    if quat[2] < 0:
                        quat = -quat
    return quat

### Conversions ###
def quat_to_euler(quat, degrees=False):
    euler = R.from_quat(quat).as_euler("xyz", degrees=degrees)
    return euler


def euler_to_quat(euler, degrees=False):
    return rot2quat(R.from_euler("xyz", euler, degrees=degrees), canonical=canonical_quat)


def rmat_to_euler(rot_mat, degrees=False):
    euler = R.from_matrix(rot_mat).as_euler("xyz", degrees=degrees)
    return euler


def euler_to_rmat(euler, degrees=False):
    return R.from_euler("xyz", euler, degrees=degrees).as_matrix()


def rmat_to_quat(rot_mat):
    quat = rot2quat(R.from_matrix(rot_mat), canonical=canonical_quat)
    return quat


def quat_to_rmat(quat):
    return R.from_quat(quat).as_matrix()


### Subtractions ###
def quat_diff(target, source):
    result = R.from_quat(target) * R.from_quat(source).inv()
    return rot2quat(result, canonical=canonical_quat)


def angle_diff(target, source, degrees=False):
    target_rot = R.from_euler("xyz", target, degrees=degrees)
    source_rot = R.from_euler("xyz", source, degrees=degrees)
    result = target_rot * source_rot.inv()
    return result.as_euler("xyz")

### Additions ###
def add_quats(delta, source):
    result = R.from_quat(delta) * R.from_quat(source)
    return rot2quat(result, canonical=canonical_quat)


def add_angles(delta, source, degrees=False):
    delta_rot = R.from_euler("xyz", delta, degrees=degrees)
    source_rot = R.from_euler("xyz", source, degrees=degrees)
    new_rot = delta_rot * source_rot
    return new_rot.as_euler("xyz", degrees=degrees)


def add_poses(delta, source, degrees=False):
    lin_sum = np.array(delta[:3]) + np.array(source[:3])
    rot_sum = add_angles(delta[3:6], source[3:6], degrees=degrees)
    result = np.concatenate([lin_sum, rot_sum])
    return result


### MISC ###
def change_pose_frame(pose, frame, degrees=False):
    R_frame = euler_to_rmat(frame[3:6], degrees=degrees)
    R_pose = euler_to_rmat(pose[3:6], degrees=degrees)
    t_frame, t_pose = frame[:3], pose[:3]
    euler_new = rmat_to_euler(R_frame @ R_pose, degrees=degrees)
    t_new = R_frame @ t_pose + t_frame
    result = np.concatenate([t_new, euler_new])
    return result