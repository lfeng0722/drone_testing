#!/usr/bin/env python3
import rospy
from airsim_simulation.sim_env import AirSimEnv
from airsim_simulation.components import Pose
import cv2
import time
import numpy as np
import os
import random
import copy
import airsim
import math

MAX_MARKER_SIZE = 1936

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return w, x, y, z

def quaternion_conjugate(q):
    w, x, y, z = q
    return w, -x, -y, -z

def get_rectangle(points):
    # for p in points[0]:
    #     print(p)
    min_x = min([point[0] for point in points])
    max_x = max([point[0] for point in points])
    min_y = min([point[1] for point in points])
    max_y = max([point[1] for point in points])

    return min_x, max_x, min_y, max_y

def get_marker_corners(marker_pose, marker_size=1):
    marker_top_left = copy.deepcopy(marker_pose)
    marker_top_left.position.x_val = marker_pose.position.x_val - marker_size / 2
    marker_top_left.position.y_val = marker_pose.position.y_val - marker_size / 2
    marker_top_right = copy.deepcopy(marker_pose)
    marker_top_right.position.x_val = marker_pose.position.x_val + marker_size / 2
    marker_top_right.position.y_val = marker_pose.position.y_val - marker_size / 2
    marker_bottom_left = copy.deepcopy(marker_pose)
    marker_bottom_left.position.x_val = marker_pose.position.x_val - marker_size / 2
    marker_bottom_left.position.y_val = marker_pose.position.y_val + marker_size / 2
    marker_bottom_right = copy.deepcopy(marker_pose)
    marker_bottom_right.position.x_val = marker_pose.position.x_val + marker_size / 2
    marker_bottom_right.position.y_val = marker_pose.position.y_val + marker_size / 2

    return [marker_top_left, marker_top_right,
            marker_bottom_left, marker_bottom_right]

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

def pos_to_pixel(drone_pose, obj_pose):
    f_x = 320
    c_x = 320
    f_y = 320
    c_y = 240

    # calculate relative position in world coordinate
    delta_x = obj_pose.position.x_val - drone_pose.position.x_val
    delta_y = obj_pose.position.y_val - drone_pose.position.y_val
    delta_z = obj_pose.position.z_val - drone_pose.position.z_val
    # print(delta_x, delta_y, delta_z)

    # convert the relative position to camera coordinate
    drone_orientation = drone_pose.orientation
    _, _, yaw = euler_from_quaternion(drone_orientation.x_val, drone_orientation.y_val,
                                    drone_orientation.z_val, drone_orientation.w_val)
    yaw_init_mat = np.array([[0, 1],[-1, 0]])
    yaw_angle_mat = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    rotation_matrix = np.linalg.inv(np.matmul(yaw_angle_mat, yaw_init_mat))

    new_delta = np.matmul(np.array([delta_x, delta_y]), rotation_matrix)
    # scale the position to pixel position
    x_img = int(f_x * new_delta[0] / delta_z + c_x)
    y_img = int(f_y * new_delta[1] / delta_z + c_y)
    # print(x_img, y_img)

    return x_img, y_img

def straight_route_collect(env, current_round=0):
    marker_id = current_round // 100

    if not os.path.exists('../datasets/sample_{}'.format(current_round)):
        os.makedirs('../datasets/sample_{}'.format(current_round))
        os.makedirs('../datasets/sample_{}/images'.format(current_round))
        os.makedirs('../datasets/sample_{}/labels'.format(current_round))

    env.reset_env()
    # set_current_weather([0] * 11)
    # drone_height = random.randint(8, 20)
    drone_height = 10

    time.sleep(2)
    label_txt = open('../datasets/sample_{}/label.txt'.format(current_round), 'w')
    # marker_id = random.randint(0, 4)

    if (current_round) % 5 == 0:
        weather_param = [random.uniform(0, 0.3)] * 11
        env.set_weather(weather_param[:9])
        env.set_time_of_day(weather_param[9:])


    # set marker at a valid position
    marker_pose = get_valid_marker(marker_id, env, drone_height=drone_height)
    marker_corners = get_marker_corners(marker_pose)
    # set the start position and end position of the drone
    drone_x = random.uniform(-3, 3)
    drone_start_y = random.uniform(-10, max(0, marker_pose.position.y_val - 10))

    # drone_end_x = drone_start_x
    drone_end_y = marker_pose.position.y_val + random.uniform(5, 15)

    current_drone_position = drone_start_y

    collected_img = 0

    drone_angle = random.randint(0, 180)

    while current_drone_position < drone_end_y:
        # client.simSetCameraPose('0', airsim.Pose(airsim.Vector3r(drone_x, current_drone_position, -10),
        #                                          airsim.to_quaternion(math.radians(-90), 0, 0)))
        drone_pose = Pose(drone_x, current_drone_position, drone_height, drone_angle)
        env.set_drone_pose(drone_pose)
        # client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(drone_x, current_drone_position, -drone_height),
        #                                      airsim.to_quaternion(0,0, math.radians(drone_angle))), True)

        # K, T = get_camera_params()
        scene_img, _ = env.get_current_scene()
        if not validate_marker_pos(env=env, set_pos=False):
            step = random.uniform(0.5, 1.5)
            current_drone_position += step
            # time.sleep(0.1)
            continue
        # semantic_img, _ = env.get_current_scene(image_type=5)

        drone_pose = env.client.simGetVehiclePose()
        corner_pixels = []
        for i in range(4):
            corner_pixels.append(pos_to_pixel(drone_pose, marker_corners[i]))

        # scene_img = image.copy()
        x1, x2, y1, y2 = get_rectangle(corner_pixels)

        corners_in_image = 0
        if (x1 < 50 or x1 > 640):
            pass
        else:
            corners_in_image += 1

        if (x2 < 50 or x2 > 640):
            pass

        else:
            corners_in_image += 1

        if (y1 < 50 or y1 > 480):
            pass

        else:
            corners_in_image += 1

        if (y2 < 50 or y2 > 480):
            pass

        else:
            corners_in_image += 1

        if corners_in_image < 3:
            pass
        else:
            print(collected_img, x1, x2, y1, y2)
            x1 = max(0, min(x1, 640))
            x2 = max(0, min(x2, 640))
            y1 = max(0, min(y1, 480))
            y2 = max(0, min(y2, 480))
            debug_img = scene_img.copy().reshape(480, 640, 3)

            labeled_image = cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            cv2.imwrite('../datasets/sample_{}/images/{}.png'.format(current_round, collected_img), scene_img)
            cv2.imwrite('../datasets/sample_{}/labels/{}.png'.format(current_round, collected_img), labeled_image)
            label_txt.write("{} {} {} {} {} {}\n".format(x1, x2, y1, y2, marker_id, drone_height))
            label_txt.flush()
            collected_img += 1

        step = random.uniform(0.5, 1.5)
        current_drone_position += step
        # time.sleep(0.1)


def get_valid_marker(marker_id, env, drone_height=10, x_range=[-6, 8], y_range=[0, 100]):
    marker_name = 'plane_marker{}'.format(marker_id)
    while True:
        marker_x = random.uniform(x_range[0], x_range[1])
        marker_y = random.uniform(y_range[0], y_range[1])
        marker_pos = Pose(marker_x, marker_y, 0)
        if validate_marker_pos(marker_pos, drone_height, env, marker_name):
            break
        else:
            time.sleep(0.5)

    # env.set_drone_pose(Pose(0, 0, 0))
    marker_pose_airsim = env.client.simGetObjectPose(marker_name)
    return marker_pose_airsim


def validate_marker_pos(marker_pos=None, drone_height=None, env=None, marker_name='plane_marker0', set_pos=True):
    if set_pos:
        env.set_marker(marker_pos, marker_name)
        env.set_drone_pose(Pose(marker_pos.x, marker_pos.y, drone_height))
        print(marker_pos.x, marker_pos.y)

        time.sleep(1)
    scene, _ = env.get_current_scene(image_type=5)

    # Reshape the image to be a 2D array where each row is an RGB triplet
    rgb_values = scene.reshape(-1, 3)

    # Get unique rows (RGB triplets) using np.unique and return_counts to get the count of each unique color
    unique_rgb_values, counts = np.unique(rgb_values, axis=0, return_counts=True)

    marker_size = 0
    # Print the unique RGB values and their respective counts
    for value, count in zip(unique_rgb_values, counts):
        if tuple(value) == marker_rgb:
            marker_size = count
            break

    print('marker size: ', marker_size)
    if marker_size / MAX_MARKER_SIZE > 0.4:
        return True
    else:
        return False


rospy.init_node("dataset_creator")

env = AirSimEnv('cv', '10.6.37.180')
color_map = env.get_segmentation_mapping()
marker_seg_id = env.set_segmentation()
marker_rgb = color_map[marker_seg_id]
print('marker RGB: ', marker_rgb)
time.sleep(1)


for i in range(300, 400):
    straight_route_collect(env, i)


rospy.spin()