
from airsim_api import *
from yolo_detector import load_model, detect
from yolov5.detect import parse_opt
import numpy as np
import random
from auto_land import pixel_to_pos, coord_convert
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')

from search_functions import cluster_data_KDTree as clustering
from scipy.spatial.distance import cdist
from obstacle_avoidance import navigate_to_goal
def search():
    mode = 'yolo'
    model = load_model()
    model = model.cuda()
    opt = parse_opt()
    global violation
    #random
    weather_soluion = np.random.random(10)

    set_current_weather(weather_soluion)
    drone_pose = respawn_drone()
    take_off(drone_pose, height=20)

    # x = local_solution[0]*10
    # y = local_solution[1]*10
    #
    # object1 = respawn_mat( mat_name='woodPallet', call_name='wood', x=x, y=y)
    #
    # print(object1 + ' is generated!')

    fly_to(0, 50, -20)
    print('going to destination')
    frame_confi=[]
    frame_coor=[]
    # setcar = setnpccarpos('Car_12')
    # # print(setcar)
    # if setcar:
    #     print('car respawn success!')

    # carmo = carmove('Car_12')
    # print(carmo)
    while True:

        image = get_current_scene(0)
        detect_result, result_img = detect(model, image, opt)



        # cv2.imshow('Detection result', result_img)
        # k = cv2.waitKey(200)
        # if k == ord('q') & 0xFF:
        #     break


        # time.sleep(0.5)
        if len(detect_result[0]) != 0:


            for h, item in enumerate(detect_result[0]):

                obj_position_c = pixel_to_pos(get_obj_pose('Copter').position.z_val, item)
                obj_position_g = coord_convert(obj_position_c, get_obj_pose('Copter').position,
                                               drone_orientation=get_obj_pose('Copter').orientation)

                frame_confi.append(item[-2].item())
                frame_coor.append(obj_position_g)


        if get_obj_pose('Copter').position.y_val > 50:
            if len(frame_confi)==0:
                break
            frame_confi=np.array(frame_confi)
            frame_coor=np.array(frame_coor)
            land_coor = clustering(frame_coor,frame_confi,thr=2)


            target_coord =np.array( [get_obj_pose('Cube_marker').position.x_val, get_obj_pose('Cube_marker').position.y_val])
            land_final =np.array( [ land_coor[0],land_coor[1]])

            distance = cdist(np.array([land_final]), np.array([target_coord]), 'euclidean')

            fitness_value=distance[0][0]
            if fitness_value>2:
                violation+=1
            print('distance',distance[0][0])
            print('flying to landing point')

            # navigate_to_goal(land_coor,'Copter')
            fly_to(land_coor[0],land_coor[1],0)
            time.sleep(20)
            # landing()
            # time.sleep(10)
            print('landing finish')
            break

    print('__________________________________________________________________________')



if __name__ == "__main__":
    budget =500
    violation = 0
    for i in range(budget):
        search()
    print('number of violation: ', violation)
