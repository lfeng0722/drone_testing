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

bound_x = [-20, 20]
bound_y = [-20, 20]
bound_z = [-15, 0]
step_size = 0.5
# Create 3D grid
x = np.arange(0, bound_x[1] - bound_x[0], step_size)
y = np.arange(0, bound_y[1] - bound_y[0], step_size)
z = np.arange(0, bound_z[1] - bound_z[0], step_size)

# Create meshgrid for 3D coordinates
# X, Y, Z = np.meshgrid(x, y, z)
# occupied = np.zeros(X.shape, dtype=bool)
grid = np.zeros((150, 150, 100))
# threshold_distance = step_size / 2.0
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
        for j in range(len(arrays)):
            if i != j:
                d= distance.euclidean(np.array(arrays[i]), np.array(arrays[j]))
                total_distance += d
    return total_distance

def plot_traj(coordinates, grid):
    for coord in coordinates:
        coord[0] -= bound_x[0]
        coord[1] -= bound_y[0]
        coord[2] -= bound_z[0]
        # print([int(coord[0]/0.5), int(coord[1]/0.5), int(coord[2]/0.5)] )
        grid[int(coord[0]/0.5), int(coord[1]/0.5), int(coord[2]/0.5)] = 1

    occupied_cells = np.argwhere(grid == 1)
    return len(occupied_cells)



    # dist = np.sqrt((X - coord[0])**2 + (Y - coord[1])**2 + (Z - coord[2])**2)
    # occupied = np.logical_or(occupied, dist <= threshold_distance)

    # Extract X, Y, and Z coordinates
    # x_coords = [point[0] - bound_x[0] for point in coordinates]
    # y_coords = [point[1] - bound_y[0] for point in coordinates]
    # z_coords = [point[2] - bound_z[0] for point in coordinates]

    # Create a new figure


    # plt.show()


exp_folder = '/home/linfeng/Documents/ga_result_lawn'



records = os.listdir(exp_folder)
traj_cover_arr = []
result_df = {'round': [], 'land_duration': [], 'land_deviation': [], 'land_success': [], 'land_fp_dist': [], 'collision': [], 'end_point_z': []}


weather_array = []
num_record=0
violation=0
top1 = 0
top5 =0
top10 = 0
found_type1 = 0
found_type_2 =0
found_type3=0
foudn_type4 = 0
maker_wrong_detect=0
type1_count =0
type2_count =0
type3_count =0
type4_count =0
for record in records:

    num_record+=1
    with open(os.path.join(exp_folder, record), 'rb') as f:
        try:
            record_result = pickle.load(f)
        except Exception as e:
            print(f"An error occurred: {e}")
            continue

        scenario = Scenario()
        scenario.load_from_json(record_result['scenario'])
        weather_array.append(scenario.weather.to_vec())

        result_df['round'].append(record.split('.')[0])
        photo = record_result['scenes']

        # print(record_result['land_result'].keys())
        for k in record_result['land_result'].keys():
            # print(k)

            result_df[k].append(record_result['land_result'][k])
        traj_cover_arr += record_result['trajectory']
        # print(record_result['marker'])
        marker = record_result['scenario']['tp_marker']
        marker_coord = [record_result['scenario']['tp_marker']['pos_x'],record_result['scenario']['tp_marker']['pos_y']]

        for item in reversed(record_result['trajectory']):
            if distance_3d(item,(0,0,0))>5:
                # print(item)
                last_avil_point = item
                break


        devi = distance_2d(item[0:2],marker_coord)
        if record_result['land_result']['collision']==1 and last_avil_point[2]<-2:
            type1_count+=1
            violation+=1
            num_photo = 0
            for item in photo:
                num_photo+=1
                img = Image.fromarray(item)
                img.save(f'/media/linfeng/HDD1/img/aviodnotsucce{num_record}_{num_photo}.jpeg')
            found_type1=1
        if record_result['land_result']['collision']==1 and last_avil_point[2]>-2:
            type2_count+=1
            violation+=1
            found_type_2=1
            num_photo = 0
            for item in photo:
                num_photo+=1
                img = Image.fromarray(item)
                img.save(f'/media/linfeng/HDD1/img/noavo{num_record}_{num_photo}.jpeg')
        if last_avil_point[2]<-8:
            type3_count+=1
            violation+=1
            found_type3=1
            num_photo = 0
            for item in photo:
                num_photo+=1
                img = Image.fromarray(item)
                img.save(f'/media/linfeng/HDD1/img/nolanding{num_record}_{num_photo}.jpeg')
        if devi>5 and last_avil_point[2]>-2 and record_result['land_result']['collision']==0:
            type4_count+=1
            violation+=1
            foudn_type4 = 1
            num_photo = 0
            for item in photo:
                num_photo+=1
                img = Image.fromarray(item)
                img.save(f'/media/linfeng/HDD1/img/landwrong{num_record}_{num_photo}.jpeg')


        if violation ==1:
            top1 =num_record
            print('top-1: ', top1)
        if violation == 5:
            top5 = num_record
            print('top-5: ', top5)
        if violation == 10:
            top10 =num_record
            print('top-10: ', top10)

        if found_type1+ found_type_2 + found_type3+ foudn_type4 > 2:
            print('round to find at least 3 types bug: ', num_record)
        if found_type1 and found_type_2 and found_type3 and foudn_type4:
            print('round to find all types bug: ', num_record)
print('Avoidance not engage: ', type1_count)
print('Not enough time to avoid: ', type2_count)
print('no landing: ', type3_count)
print('wrong landing: ', type4_count)
print('violation: ',violation)
print('violation_Rate: ',violation/num_record)
print('parameter distance: ', compute_total_distance(weather_array)/len(weather_array))

# result_df = pd.DataFrame(result_df)
cover = plot_traj(traj_cover_arr, grid)
print('coverage: ', cover/num_record)
# for record in records:
#     # print(record)
#     # print(record_result['trajectory'])
#     # break
#     with open(os.path.join(exp_folder, record), 'rb') as f:
#
#         record_result = pickle.load(f)
#         # if record_result['land_result']['land_success'] == 0:
#         #     print(record)
#         #     print(record_result['scenario'])
#             # break
#         result_df['round'].append(record.split('.')[0])
#         for k in record_result['land_result'].keys():
#             result_df[k].append(record_result['land_result'][k])
#         traj_cover_arr += record_result['trajectory']
#         result_df['end_point_z'].append(record_result['trajectory'][-1][2])
        # print(record_result.keys())
        # print(record_result['land_result'])
    # break
#

# print('number of violation: ', result_df[(result_df['land_deviation']>2) & result_df['end_point_z']>2].shape[0]+result_df[(result_df['land_deviation']<1.5) & (result_df['collision']==1)].shape[0])
# print('number of simulation: ',  result_df.shape[0])
# print('number of wrong landing: ', maker_wrong_detect)
# print('number of collision: ', result_df[result_df['collision']==1].shape[0])
# print('number of collision-try to aviod not success: ', result_df[(result_df['collision']==1)&(result_df['end_point_z']<-2)].shape[0])
# print('number of collision-no avoid: ', result_df[(result_df['collision']==1)&(result_df['end_point_z']>-2)].shape[0])
# # print('number of unland-mark not detected: ', result_df[(result_df['land_success']==0) & (result_df['land_deviation']>4)].shape[0])
# # print(result_df['end_point_z'])
# print('number of unland-mark not detected: ', result_df[(result_df['end_point_z'] < -8)].shape[0])
# print('number of unland-mark wrong detected: ', result_df[(result_df['land_deviation'] > 3) & (result_df['end_point_z'] > -3) & (result_df['collision']==0)].shape[0])
# # print('percentage of violation: ', violation/result_df.shape[0])
    # print(result_df[(result_df['land_deviation'] > 3) & (result_df['end_point_z'] > -2)])
    # print(result_df[(result_df['end_point_z'] < -5) & (result_df['land_deviation'] > 3)])
    # print('number of collision: ', result_df[result_df['collision'] == 1].shape[0])
    # print('number of collision-unland: ', result_df[(result_df['collision']==1)& (result_df['land_success']==0)& (result_df['land_deviation']<3)].shape[0])
    # print('number of collision-land: ', result_df[(result_df['collision']==1)& (result_df['land_success']==1)& (result_df['land_deviation']<2)].shape[0])
    # print(result_df[(result_df['land_deviation'] <= 1.5)].shape[0])
    # print(result_df[(result_df['land_deviation'] <= 1.5) & (result_df['end_point_z'] < -2)].shape[0])
    # print('number of inaccurately land: ', result_df[(result_df['land_deviation'] > 1.5) & (result_df['land_deviation'] <= 3) & (result_df['end_point_z'] < -2)].shape[0])
    # print('number of wrong land: ', result_df[(result_df['land_deviation'] > 3) & (result_df['end_point_z'] < -2)].shape[0])

    # # print('number of unland-mark hide by objects or weather: ', result_df[(result_df['land_success']==0) & (result_df['land_deviation']<3)].shape[0])

    # # print('number of inaccurate landing: ', result_df[(result_df['land_deviation']>1.5) & (result_df['land_deviation']<3) & (result_df['land_success']==1)].shape[0])
    # # print('number of false landing: ', result_df[(result_df['land_deviation']>3) & (result_df['land_success']==1)].shape[0])
    # array.append(result_df[(result_df['land_success']==0) & (result_df['land_deviation']>4)].shape[0])
    # array.append(result_df[(result_df['land_success']==0) & (result_df['land_deviation']<3)].shape[0])
    # array.append(result_df[(result_df['collision']==1)& (result_df['land_success']==0)& (result_df['land_deviation']<3)].shape[0])
    # array.append(result_df[(result_df['collision']==1)& (result_df['land_success']==1)& (result_df['land_deviation']<2)].shape[0])

# big_categories = ['Ground', 'Lawn']
# values = [ground,lawn]
#
# N=4
# width = 0.15  #
# group_width = N * width  # 一个大类的总宽度
# ind = np.arange(N) * group_width  # x轴的位置
# fig, ax = plt.subplots()
#
# bars = []
# for i, val in enumerate(values):
#     positions = ind + i * width  # 每个大类的x轴位置
#     bar = ax.bar(positions, val, width)
#     bars.append(bar)
# ax.axhline(0, color='grey', linewidth=0.8)
# ax.set_ylabel('number of cases')
# # ax.set_title('每个大类里面的小类的值')
# ax.set_xticks(ind + 1.5 * width)
# ax.set_xticklabels(('type1', 'type2', 'type3', 'type4'))
# ax.legend([b[0] for b in bars], big_categories)
#
# # 显示图
# plt.show()