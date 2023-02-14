import numpy as np
from airsim import *
import time
from airsim_api import fly_to
# 计算飞机前方障碍物的位置
def get_obstacle_position(client):
    lidar_data = client.getLidarData()
    point_cloud = np.array(lidar_data.point_cloud)

    # 找到前方障碍物的位置
    front_indices = np.where((point_cloud[:,0] > 0) & (np.abs(point_cloud[:,1]) < 5))
    front_obstacle = np.min(point_cloud[front_indices, 2])

    if front_obstacle < 10:
        # 计算前方障碍物的位置
        obstacle_position = np.mean(point_cloud[front_indices, :2], axis=0)
        return obstacle_position
    else:
        return None

# 控制飞机绕开障碍物
def avoid_obstacle(client, obstacle_position):
    if obstacle_position is not None:
        yaw_rate = np.arctan2(obstacle_position[1], obstacle_position[0]) * 180 / np.pi
        client.moveByAngleZ(-5, yaw_rate, -10)
        time.sleep(1)
    else:
        client.moveByAngleZ(5, 0, 10)
        time.sleep(1)

def navigate_to_goal(goal, vehicle_name):
    client = MultirotorClient()
    client.confirmConnection()
    # client.simSetVehiclePose(Pose(Vector3r(0, 0, 10), Quaternionr(0, 0, 0, 1)), True, vehicle_name)
    fly_to(goal[0],goal[1],0)

    while True:
        # 获取当前飞机位置
        pos = client.simGetVehiclePose(vehicle_name).position

        # 判断飞机是否到达目标点
        if np.linalg.norm(np.array([goal[0], goal[1]]) - np.array([pos.x_val, pos.y_val])) < 2:
            break

        # 计算前方障碍物的位置
        obstacle_position = get_obstacle_position(client)
        avoid_obstacle(client, obstacle_position)