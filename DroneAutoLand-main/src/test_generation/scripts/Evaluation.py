#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import os
import pickle
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from airsim_simulation.scenario import Scenario
from scipy.spatial import distance
from PIL import Image
import math
from airsim_simulation.configuration import map_config


map_name = 'lawn'
bound_x = map_config[map_name]['random_pos_x']
bound_y = map_config[map_name]['random_pos_y']
bound_z = [-30, 0]
grid_size = 2

grid = np.zeros(((bound_x[1]-bound_x[0])//2, (bound_y[1]-bound_y[0])//2, 15))

def distance_2d(point1, point2):
    """Calculate the Euclidean distance between two points in 2D space."""
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def min_max_normalization(data):
    """
    Perform min-max normalization on the input data.

    Parameters:
    - data: NumPy array of shape (n_samples, n_features)

    Returns:
    - Normalized data
    """
    min_vals = np.min(data, axis=0)
    max_vals = np.max(data, axis=0)

    normalized_data = (data - min_vals) / (max_vals - min_vals)

    return normalized_data

def distance_3d(point1, point2):
    """
    计算三维空间中两点之间的距离。

    :param point1: 第一个点的坐标，一个包含三个元素的元组 (x1, y1, z1)。
    :param point2: 第二个点的坐标，一个包含三个元素的元组 (x2, y2, z2)。
    :return: 两点之间的距离。
    """
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def compute_total_distance(arrays):
    total_distance = 0
    for i in range(len(arrays)):
        element_dis=0
        for j in range(len(arrays)):
            if i != j:
                d= distance.euclidean(np.array(arrays[i]), np.array(arrays[j]))
                element_dis += d
        total_distance+= element_dis/len(arrays)
    return total_distance

def plot_traj(coordinates, grid):
    for coord in coordinates:

        coord[0] -= bound_x[0]
        coord[1] -= bound_y[0]
        coord[2] -= bound_z[0]

        if abs(int(coord[0]/2)) <(bound_x[1]-bound_x[0])//2 and abs(int(coord[1]/2))<(bound_y[1]-bound_y[0])//2 and abs(int(coord[2]/2))<15:
            grid[int(coord[0]/2), int(coord[1]/2), int(coord[2]/2)] = 1

    occupied_cells = np.argwhere(grid == 1)
    return len(occupied_cells)



exp_folder = '/media/linfeng/HDD1/NEW_ICSE/GA_MM_Lawn_01'



records = os.listdir(exp_folder)
traj_cover_arr = []
result_df = {'round': [], 'land_duration': [], 'land_deviation': [], 'land_success': [], 'land_fp_dist': [], 'collision': [], 'end_point_z': []}


weather_array = []
num_record=0
violation=0
top1 = 0
top5 =0
top10 = 0
round_found_all_bug = []
collision_type = 0
wrong_land_type = 0
found_collision_type = 0
found_wrong_land_type = 0

for record in records:
    if_vio = 0
    num_record+=1

    with open(os.path.join(exp_folder, record), 'rb') as f:
        try:
            record_result = pickle.load(f)
        except Exception as e:
            print(f"An error occurred: {e}")
            continue

        scenario = Scenario()
        scenario.load_from_json(record_result['scenario'])


        result_df['round'].append(record.split('.')[0])
        photo = record_result['scenes']

     
        for k in record_result['land_result'].keys():
            # print(k)

            result_df[k].append(record_result['land_result'][k])

        
        destination = [record_result['scenario']['gps_pose']['pos_x'],record_result['scenario']['gps_pose']['pos_y']]
        
        marker = record_result['scenario']['tp_marker']
        marker_coord = [record_result['scenario']['tp_marker']['pos_x'],record_result['scenario']['tp_marker']['pos_y']]
        
        marker_destination = distance_2d(destination,marker_coord)
        
        for item in reversed(record_result['trajectory']):
            if distance_3d(item,(0,0,0))>5:
                # print(item)
                last_avil_point = item
                break
        if marker_destination<10:
            weather_vec = scenario.weather.to_vec()
            modified_weather_vec = [0.15 if item > 0.15 else 0 if item < 0 else item for item in weather_vec]
            weather_array.append(modified_weather_vec)

        devi = distance_2d(last_avil_point[0:2],marker_coord)
        
        if record_result['land_result']['collision'] == 1:
            # print(record_result['land_result']['collision'])
            collision_type+=1
            violation+=1
            num_photo = 0
            for item in photo:
                num_photo+=1
                img = Image.fromarray(item)
                img.save(f'/media/linfeng/HDD1/img/collision{num_record}_{num_photo}.jpeg')
            found_collision_type=1
            if_vio=1
 
        if record_result['land_result']['land_deviation'] >1.5 and record_result['land_result']['collision']==0:
            violation+=1
            found_wrong_land_type = 1
            num_photo = 0
            for item in photo:
                num_photo+=1
                img = Image.fromarray(item)
                img.save(f'/media/linfeng/HDD1/img/landwrong{num_record}_{num_photo}.jpeg')

            if_vio=1
        if if_vio:
            traj_cover_arr += record_result['trajectory']
        if violation ==1:
            top1 = num_record
        if violation == 5:
            top5 = num_record
        if violation == 10:
            top10 =num_record
        if found_wrong_land_type and found_collision_type:
            round_found_all_bug.append(num_record)
print('top-1: ', top1)
print('top-5: ', top5)
print('top-10: ', top10)
if len(round_found_all_bug)!=0:
    print('round to find all bugs', min(round_found_all_bug))
else:
    print('cannot find all violation types')
print('collision:', collision_type)
print('wrong landing: ', wrong_land_type)
print('violation: ',violation)
print('violation_Rate: ',violation/num_record)
print('parameter distance: ', compute_total_distance(weather_array)/len(weather_array))

cover = plot_traj(traj_cover_arr, grid)
print('coverage: ', cover/((bound_x[1]-bound_x[0])//2* (bound_y[1]-bound_y[0])//2* 15))

