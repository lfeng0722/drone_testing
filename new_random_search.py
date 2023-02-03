from airsim_api import *
from yolo_detector import load_model, detect
from yolov5.detect import parse_opt
import numpy as np
import random
from auto_land import pixel_to_pos, coord_convert
from scipy.spatial import cKDTree as KDTree
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')




def cluster_data_KDTree(a, thr=0.1):
    t = KDTree(a)
    mask = np.ones(a.shape[:1], bool)
    idx = 0
    nxt = 1
    while nxt:
        mask[t.query_ball_point(a[idx], thr)] = False
        nxt = mask[idx:].argmax()
        mask[idx] = True
        idx += nxt
    return a[mask]

mode = 'yolo'
model = load_model()
model = model.cuda()
opt = parse_opt()
search_time =20

for j in range(search_time):
    print('run: ' + str(j))
    potential_landing=[]
    # params = np.random.random(11)
    # set_current_weather(params)

    drone_pose = respawn_drone()
    take_off(drone_pose, height=20)
    fly_to(0, 50,-20)
    print('going to destination')
    while True:
        image = get_current_scene(0)
        detect_result,result_img = detect(model, image, opt)
        if len(detect_result[0]) !=0:
            for item in detect_result[0]:
                if item[-2]>=0.6:
                    obj_position_c = pixel_to_pos(get_obj_pose('Copter').position.z_val, item)
                    obj_position_g = coord_convert(obj_position_c, get_obj_pose('Copter').position, drone_orientation=get_obj_pose('Copter').orientation)
                    potential_landing.append(obj_position_g)
        if  get_obj_pose('Copter').position.y_val>50:
            break


    fly_to(0,0, -20)
    print('returning')
    while True:
        image = get_current_scene(0)
        detect_result,result_img = detect(model, image, opt)
        if len(detect_result[0]) !=0:
            for item in detect_result[0]:
                if item[-2]>=0.6:
                    obj_position_c = pixel_to_pos(get_obj_pose('Copter').position.z_val, item)
                    obj_position_g = coord_convert(obj_position_c, get_obj_pose('Copter').position, drone_orientation=get_obj_pose('Copter').orientation)
                    potential_landing.append(obj_position_g)
        if  get_obj_pose('Copter').position.y_val<0.5:
            break

    potential_landing=  np.array(potential_landing)
    clustered_points = cluster_data_KDTree(potential_landing, thr=2)
    for point in clustered_points:
        print(point)
        fly_to(point[0],point[1],-20)
        time.sleep(10)
        image = get_current_scene(0)
        detect_result,result_img = detect(model, image, opt)
        # print(detect_result[0])
        cv2.imshow('Detection result', result_img)
        cv2.waitKey(0)

    # print(len(potential_landing))
    # print(out1)
    # for point in potential_landing:
    #
    #     plt.plot(point[0],point[1],'x')
    #     plt.axis([-100,100,-100,100])
    # plt.show()