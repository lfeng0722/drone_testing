import cv2
import numpy as np
from marker_detection.result import PoseResult, Position, Rotation

def quaternion_to_rotation_matrix(q):
    # Extract components from quaternion
    qw, qx, qy, qz = q

    # Compute elements for the rotation matrix
    R = np.zeros((3, 3))
    R[0, 0] = 1 - 2 * qy ** 2 - 2 * qz ** 2
    R[0, 1] = 2 * qx * qy - 2 * qw * qz
    R[0, 2] = 2 * qx * qz + 2 * qw * qy

    R[1, 0] = 2 * qx * qy + 2 * qw * qz
    R[1, 1] = 1 - 2 * qx ** 2 - 2 * qz ** 2
    R[1, 2] = 2 * qy * qz - 2 * qw * qx

    R[2, 0] = 2 * qx * qz - 2 * qw * qy
    R[2, 1] = 2 * qy * qz + 2 * qw * qx
    R[2, 2] = 1 - 2 * qx ** 2 - 2 * qy ** 2

    return R

def transformation_matrix(R, t):
    # Initialize a 4x4 identity matrix
    T = np.identity(4)

    # Place the rotation matrix and translation vector into T
    T[:3, :3] = R
    T[:3, 3] = t

    return T

def pixel_to_coord(x, y, d, T, K):
    # 1. Convert pixel position and depth to camera frame
    uv1 = np.linalg.inv(K).dot([x, y, 1]) * d
    X, Y, Z = uv1[0], uv1[1], d

    # 2. Homogeneous coordinate in camera frame
    P_camera = np.array([X, Y, Z, 1]).reshape(4, 1)

    # 3. Convert camera frame to drone frame
    # P_drone = np.matmul(T_camera_to_drone, P_camera)

    # 4. Convert drone frame to world frame
    P_world = np.matmul(T, P_camera)
    # print(P_world)
    coord = [P_world[0][0], P_world[1][0], P_world[2][0]]

    return coord


class DetectionProcessor(object):
    def __init__(self, camera_intrinsics=np.array([[320, 0.0, 320.0], [0.0, 320, 240.0], [0.0, 0.0, 1.0]]), marker_size=2):
        self.camera_intrinsics = camera_intrinsics
        self.marker_size = marker_size

    """
    Convert the detected marker in image to world coordinates
    """
    def process_result(self, pixel_result, camera_pose):
        # print(pixel_result.marker_center, pixel_result.marker_corners)
        # print(camera_pose)
        # print()
        t = [camera_pose.pose.position.x, camera_pose.pose.position.y, camera_pose.pose.position.z]
        orientation = [camera_pose.pose.orientation.w, camera_pose.pose.orientation.x,
                        camera_pose.pose.orientation.y, camera_pose.pose.orientation.z]
        R = quaternion_to_rotation_matrix(orientation)
        T = transformation_matrix(R, t)
        marker_center = pixel_result.marker_center

        # current coord is NED corrdinate
        coord = pixel_to_coord(marker_center[0], marker_center[1], -t[2], T, self.camera_intrinsics)
        # print(coord)
        result = PoseResult(pixel_result.marker_id, Position(*coord), Rotation(), pixel_result.confidence, pixel_result.debug_image)
        return result