from airsim_api import *
from yolo_detector import load_model, detect
from yolov5.detect import parse_opt
import numpy as np
import random
from auto_land import pixel_to_pos, coord_convert
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')



mode = 'yolo'
model = load_model()
model = model.cuda()
opt = parse_opt()
highest_score =-1
search_time =1000
run_score_list =[]
fp_num = 0
tn_num = 0
max_object_num = 5







#start run loop
for j in range(search_time):
    print('run: ' + str(j))
    run_score=0
    params = np.random.random(11)
    set_current_weather(params)
    drone_pose = respawn_drone()
    take_off(drone_pose, height=20)

    fly_to(0, 50, -20)
    print('going to destination')
    while True:
        # object_density = 0
        image = get_current_scene(0)
        detect_result,result_img = detect(model, image, opt)
        time.sleep(0.5)
        if len(detect_result[0]) !=0:
            for h, item in enumerate(detect_result[0]):
                # local_search_time =10
                obj_position_c = pixel_to_pos(get_obj_pose('Copter').position.z_val, item)
                #centroid of detected object
                obj_position_g = coord_convert(obj_position_c, get_obj_pose('Copter').position, drone_orientation=get_obj_pose('Copter').orientation)
                # print('1111',item[-2])
                print(obj_position_g[1])
                print(get_obj_pose('Cube_marker').position.y_val)
                #TN
                if abs(obj_position_g[1]-get_obj_pose('Cube_marker').position.y_val) < 1 and abs(obj_position_g[0]-get_obj_pose('Cube_marker').position.x_val)< 1 and item[-2]<0.8:
                    tn_num+=1
                    print('true negtive detected in this image')
                    # print('Coordinate of True:',obj_position_g)
                    cv2.imwrite(f'True_negative/tn_run{j}img{tn_num}.jpg',image)
                    run_score+=item[-2]
                elif abs(obj_position_g[1]-get_obj_pose('Cube_marker').position.y_val) < 1 and abs(obj_position_g[0]-get_obj_pose('Cube_marker').position.x_val)< 1 and item[-2]>=0.8 and item[-2]-0.8<=0.1:

                    print('potential true negtive detected in this image, local search start')
                    fly_to(obj_position_g[0], obj_position_g[1], -20)
                    time.sleep(5)
                    # tn_num+=1
                    count=0
                    for n in range(max_object_num):
                        time.sleep(1)
                        x = random.uniform(-1,1)
                        y = random.uniform(-1,1)

                        object1 = respawn_mat(location =obj_position_g,mat_name='Autumn_Leaves', call_name = 'leaf'+ str(n), x=x, y=y)
                        count +=1
                        print(f'{object1} has created')
                        image_local_search = get_current_scene(0)
                        detect_result_local,result_img_local = detect(model, image_local_search, opt)
                        found_tn_local=False
                        if len(detect_result_local[0]) !=0:

                            for item_local in detect_result_local[0]:
                                # found_tn_local=False
                                obj_position_c_local = pixel_to_pos(get_obj_pose('Copter').position.z_val, item_local)
                                obj_position_g_local = coord_convert(obj_position_c_local, get_obj_pose('Copter').position, drone_orientation=get_obj_pose('Copter').orientation)
                                if abs(obj_position_g_local[1]-get_obj_pose('Cube_marker').position.y_val) < 1 and abs(obj_position_g_local[0]-get_obj_pose('Cube_marker').position.x_val)< 1 and item_local[-2]<0.8:
                                    found_tn_local =True
                                    tn_num+=1
                                    print('After local search, true negative example found')
                                    cv2.imwrite(f'True_negative/tn_local_run_{j}img_{tn_num}.jpg',image)
                                    run_score+=item_local[-2]
                                    break
                                # else:
                                #     print('true negative example not found, add another objects')
                            if found_tn_local:
                                break
                            else:
                                print('true negative is not found in this time')
                    if found_tn_local== False:
                        print('After local search, true negative cannot be found')
                    # time.sleep(1)
                        # if n ==max_object_num and found_tn_local ==False:

                    print('Local search end, destory generated objects')
                    for l in range(count):
                        des_re = destroy_mat('leaf'+ str(l))
                        if des_re:
                            print('object ' + str(l) +' is destroyed')
                        time.sleep(1)

                    fly_to(0, 50, -20)
                    # for n in range(local_search_time):
                    #     object_density = random.randint(1,5)
                    #     image_local_search = get_current_scene(0)
                    #     for i in range(object_density):
                    #         x = random.uniform(-1,1)
                    #         y = random.uniform(-1,1)
                    #         object1 = respawn_mat(location ='Cube_marker',mat_name='Autumn_Leaves', call_name = 'leaf'+ str(i), x=x, y=y)
                    # if object_density < 5:
                    #     object_density+=1
                    #     x = random.uniform(-1,1)
                    #     y = random.uniform(-1,1)
                    #     object1 = respawn_mat(location ='Cube_marker',mat_name='Autumn_Leaves', call_name = 'leaf'+ str(object_density), x=x, y=y)
                    #     print(f'object{object_density} generated')
                    #     image_local_search = get_current_scene(0)
                    #     detect_result,result_img = detect(model, image_local_search, opt)
                    #     if detect_result[0][h][-2]<0.8:
                    #         print('After local search, true negtive detected in this image')
                    #         cv2.imwrite(f'True_negative/tn_run{j}img{fp_num}.jpg',image)
                    #         run_score+=1

                    # else:
                    #     print('local search again')
                    # for n in range(local_search_time):
                    #     object_density = random.randint(1,5)
                    #     image_local_search = get_current_scene(0)
                    #     for i in range(object_density):
                    #         x = random.uniform(-1,1)
                    #         y = random.uniform(-1,1)
                    #         object1 = respawn_mat(location ='Cube_marker',mat_name='Autumn_Leaves', call_name = 'leaf'+ str(i), x=x, y=y)
                    #     print(str(object_density)+' object generated')
                    #     detect_result,result_img = detect(model, image_local_search, opt)
                    #     if detect_result[0][h][-2]<0.8:
                    #         print('After local search, true negtive detected in this image')
                    #         cv2.imwrite(f'True_negative/tn_run{j}img{fp_num}.jpg',image)
                    #         run_score+=1
                    #         break
                    #     else:
                    #         print('local search again')
                    # fly_to(0, 50, -50)

                elif abs(obj_position_g[1]-get_obj_pose('Cube_marker').position.y_val) >= 2 and abs(obj_position_g[0]-get_obj_pose('Cube_marker').position.x_val)>= 2 and item[-2]>0.8:
                    # else:
                    fp_num+=1
                    run_score+=item[-2]
                    print('False positive detected in this image')
                    # print('Coordinate of False:',obj_position_g)
                    cv2.imwrite(f'False_positive/fp_run{j}img{fp_num}.jpg',image)
                elif abs(obj_position_g[1]-get_obj_pose('Cube_marker').position.y_val) >= 2 and abs(obj_position_g[0]-get_obj_pose('Cube_marker').position.x_val)>= 2 and 0.8-item[-2]<0.1:
                    print('potential False Positive detected in this image, local search start')
                    # fp_num+=1
                    fly_to(obj_position_g[0], obj_position_g[1], -20)
                    time.sleep(5)
                    count = 0
                    for n in range(max_object_num):
                        time.sleep(1)
                        x = random.uniform(-1,1)
                        y = random.uniform(-1,1)

                        object1 = respawn_mat(location =obj_position_g,mat_name='Autumn_Leaves', call_name = 'leaf'+ str(n), x=x, y=y)
                        count +=1
                        print(f'{object1} has created')
                        image_local_search = get_current_scene(0)
                        detect_result_local,result_img_local = detect(model, image_local_search, opt)
                        found_fp_local=False
                        if len(detect_result_local[0]) !=0:

                            for item_local in detect_result_local[0]:
                                # found_fp_local=False
                                obj_position_c_local = pixel_to_pos(get_obj_pose('Copter').position.z_val, item_local)
                                obj_position_g_local = coord_convert(obj_position_c_local, get_obj_pose('Copter').position, drone_orientation=get_obj_pose('Copter').orientation)
                                if abs(obj_position_g_local[1]-get_obj_pose('Cube_marker').position.y_val) >= 2 and abs(obj_position_g_local[0]-get_obj_pose('Cube_marker').position.x_val)>= 2 and item_local[-2]>0.8:
                                    found_fp_local =True
                                    fp_num+=1
                                    print('After local search, False positive example found')
                                    cv2.imwrite(f'False_positive/fp_local_run_{j}img_{tn_num}.jpg',image)
                                    run_score+=item_local[-2]
                                    break
                                # else:
                                #     print('true negative example not found, add another objects')
                            if found_fp_local:
                                break
                            else:
                                print('False positive is not found in this time')
                        else:
                            print('False positive disapeared')
                    if found_fp_local ==False:
                        print('After local search, False positive cannot be found')
                        # time.sleep(1)
                        # if n ==max_object_num and found_fp_local ==False:
                        #     print('After local search, False positive cannot be found')
                    print('Local search end, destory generated objects')
                    for l in range(count):
                        des_re = destroy_mat('leaf'+ str(l))
                        if des_re:
                            print('object ' + str(l) +' is destroyed')
                        time.sleep(1)

                    fly_to(0, 50, -20)

        if  get_obj_pose('Copter').position.y_val>50:
                break
    # fly_to(0, 0, -50)
    # print('Return')
    # while True:
    #     # object_density = 0
    #     image = get_current_scene(0)
    #     detect_result,result_img = detect(model, image, opt)
    #     time.sleep(0.5)
    #     if len(detect_result[0]) !=0:
    #         for h, item in enumerate(detect_result[0]):
    #             # local_search_time =10
    #             obj_position_c = pixel_to_pos(get_obj_pose('Copter').position.z_val, item)
    #             #centroid of detected object
    #             obj_position_g = coord_convert(obj_position_c, get_obj_pose('Copter').position, drone_orientation=get_obj_pose('Copter').orientation)
    #             # print('1111',item[-2])
    #
    #             #TN
    #             if abs(obj_position_g[1]-get_obj_pose('Cube_marker').position.y_val) < 1 and abs(obj_position_g[0]-get_obj_pose('Cube_marker').position.x_val)< 1 and item[-2]<0.8:
    #                 tn_num+=1
    #                 print('true negtive detected in this image')
    #                 # print('Coordinate of True:',obj_position_g)
    #                 cv2.imwrite(f'True_negative/tn_run{j}img{fp_num}.jpg',image)
    #                 run_score+=item[-2]
    #             elif abs(obj_position_g[1]-get_obj_pose('Cube_marker').position.y_val) < 1 and abs(obj_position_g[0]-get_obj_pose('Cube_marker').position.x_val)< 1 and item[-2]>=0.8 and item[-2]-0.8<=0.1:
    #
    #                 print('potential true negtive detected in this image, local search start')
    #                 fly_to(obj_position_g[0], obj_position_g[1], -50)
    #                 time.sleep(3)
    #                 # tn_num+=1
    #                 count=0
    #                 for n in range(max_object_num):
    #                     time.sleep(1)
    #                     x = random.uniform(-1,1)
    #                     y = random.uniform(-1,1)
    #
    #                     object1 = respawn_mat(location =obj_position_g,mat_name='Autumn_Leaves', call_name = 'leaf'+ str(n), x=x, y=y)
    #                     count +=1
    #                     print(f'{object1} has created')
    #                     image_local_search = get_current_scene(0)
    #                     detect_result_local,result_img_local = detect(model, image_local_search, opt)
    #                     found_tn_local=False
    #                     if len(detect_result_local[0]) !=0:
    #
    #                         for item_local in detect_result_local[0]:
    #                             # found_tn_local=False
    #                             obj_position_c_local = pixel_to_pos(get_obj_pose('Copter').position.z_val, item_local)
    #                             obj_position_g_local = coord_convert(obj_position_c_local, get_obj_pose('Copter').position, drone_orientation=get_obj_pose('Copter').orientation)
    #                             if abs(obj_position_g_local[1]-get_obj_pose('Cube_marker').position.y_val) < 1 and abs(obj_position_g_local[0]-get_obj_pose('Cube_marker').position.x_val)< 1 and item_local[-2]<0.8:
    #                                 found_tn_local =True
    #                                 tn_num+=1
    #                                 print('After local search, true negative example found')
    #                                 cv2.imwrite(f'True_negative/tn_local_run_{j}img_{tn_num}.jpg',image)
    #                                 run_score+=item_local[-2]
    #                                 break
    #                             # else:
    #                             #     print('true negative example not found, add another objects')
    #                         if found_tn_local:
    #                             break
    #                         else:
    #                             print('true negative is not found in this time')
    #                 if found_tn_local== False:
    #                     print('After local search, true negative cannot be found')
    #                 # time.sleep(1)
    #                 # if n ==max_object_num and found_tn_local ==False:
    #
    #                 print('Local search end, destory generated objects')
    #                 for l in range(count):
    #                     des_re = destroy_mat('leaf'+ str(l))
    #                     if des_re:
    #                         print('object ' + str(l) +' is destroyed')
    #                     time.sleep(1)
    #
    #                 fly_to(0, 0, -50)
    #                 # for n in range(local_search_time):
    #                 #     object_density = random.randint(1,5)
    #                 #     image_local_search = get_current_scene(0)
    #                 #     for i in range(object_density):
    #                 #         x = random.uniform(-1,1)
    #                 #         y = random.uniform(-1,1)
    #                 #         object1 = respawn_mat(location ='Cube_marker',mat_name='Autumn_Leaves', call_name = 'leaf'+ str(i), x=x, y=y)
    #                 # if object_density < 5:
    #                 #     object_density+=1
    #                 #     x = random.uniform(-1,1)
    #                 #     y = random.uniform(-1,1)
    #                 #     object1 = respawn_mat(location ='Cube_marker',mat_name='Autumn_Leaves', call_name = 'leaf'+ str(object_density), x=x, y=y)
    #                 #     print(f'object{object_density} generated')
    #                 #     image_local_search = get_current_scene(0)
    #                 #     detect_result,result_img = detect(model, image_local_search, opt)
    #                 #     if detect_result[0][h][-2]<0.8:
    #                 #         print('After local search, true negtive detected in this image')
    #                 #         cv2.imwrite(f'True_negative/tn_run{j}img{fp_num}.jpg',image)
    #                 #         run_score+=1
    #
    #                 # else:
    #                 #     print('local search again')
    #                 # for n in range(local_search_time):
    #                 #     object_density = random.randint(1,5)
    #                 #     image_local_search = get_current_scene(0)
    #                 #     for i in range(object_density):
    #                 #         x = random.uniform(-1,1)
    #                 #         y = random.uniform(-1,1)
    #                 #         object1 = respawn_mat(location ='Cube_marker',mat_name='Autumn_Leaves', call_name = 'leaf'+ str(i), x=x, y=y)
    #                 #     print(str(object_density)+' object generated')
    #                 #     detect_result,result_img = detect(model, image_local_search, opt)
    #                 #     if detect_result[0][h][-2]<0.8:
    #                 #         print('After local search, true negtive detected in this image')
    #                 #         cv2.imwrite(f'True_negative/tn_run{j}img{fp_num}.jpg',image)
    #                 #         run_score+=1
    #                 #         break
    #                 #     else:
    #                 #         print('local search again')
    #                 # fly_to(0, 50, -50)
    #             elif abs(obj_position_g[1]-get_obj_pose('Cube_marker').position.y_val) >= 1 and abs(obj_position_g[0]-get_obj_pose('Cube_marker').position.x_val)>= 1 and item[-2]>0.8:
    #                 # else:
    #                 fp_num+=1
    #                 run_score+=item[-2]
    #                 print('False positive detected in this image')
    #                 # print('Coordinate of False:',obj_position_g)
    #                 cv2.imwrite(f'False_positive/fp_run{j}img{fp_num}.jpg',image)
    #             elif abs(obj_position_g[1]-get_obj_pose('Cube_marker').position.y_val) >= 1 and abs(obj_position_g[0]-get_obj_pose('Cube_marker').position.x_val)>= 1 and 0.8-item[-2]<0.1:
    #                 print('potential False Positive detected in this image, local search start')
    #                 # fp_num+=1
    #                 fly_to(obj_position_g[0], obj_position_g[1], -50)
    #                 time.sleep(3)
    #                 count = 0
    #                 for n in range(max_object_num):
    #                     time.sleep(1)
    #                     x = random.uniform(-1,1)
    #                     y = random.uniform(-1,1)
    #
    #                     object1 = respawn_mat(location =obj_position_g,mat_name='Autumn_Leaves', call_name = 'leaf'+ str(n), x=x, y=y)
    #                     count +=1
    #                     print(f'{object1} has created')
    #                     image_local_search = get_current_scene(0)
    #                     detect_result_local,result_img_local = detect(model, image_local_search, opt)
    #                     found_fp_local=False
    #                     if len(detect_result_local[0]) !=0:
    #
    #                         for item_local in detect_result_local[0]:
    #                             # found_fp_local=False
    #                             obj_position_c_local = pixel_to_pos(get_obj_pose('Copter').position.z_val, item_local)
    #                             obj_position_g_local = coord_convert(obj_position_c_local, get_obj_pose('Copter').position, drone_orientation=get_obj_pose('Copter').orientation)
    #                             if abs(obj_position_g_local[1]-get_obj_pose('Cube_marker').position.y_val) >= 1 and abs(obj_position_g_local[0]-get_obj_pose('Cube_marker').position.x_val)>= 1 and item_local[-2]>0.8:
    #                                 found_fp_local =True
    #                                 print('After local search, False positive example found')
    #                                 cv2.imwrite(f'False_positive/fp_local_run_{j}img_{tn_num}.jpg',image)
    #                                 run_score+=item_local[-2]
    #                                 break
    #                             # else:
    #                             #     print('true negative example not found, add another objects')
    #                         if found_fp_local:
    #                             break
    #                         else:
    #                             print('False positive is not found in this time')
    #                     # else:
    #                 if found_fp_local ==False:
    #                     print('After local search, False positive cannot be found')
    #                     # time.sleep(1)
    #                     # if n ==max_object_num and found_fp_local ==False:
    #                     #     print('After local search, False positive cannot be found')
    #                 print('Local search end, destory generated objects')
    #                 for l in range(count):
    #                     des_re = destroy_mat('leaf'+ str(l))
    #                     if des_re:
    #                         print('object ' + str(l) +' is destroyed')
    #                     time.sleep(1)
    #
    #                 fly_to(0, 0, -50)
    #     if  get_obj_pose('Copter').position.y_val<0.5:
    #         break

    # for m in range(object_density):
    #     des_re = destroy_mat('leaf'+ str(m))
    #     if des_re:
    #         print('object ' + str(m) +' is destroyed')
    if run_score != 0:
        run_score= run_score.cpu().numpy()

    print('score:',run_score)
    run_score_list.append(run_score)
    print('________________________________________________________')
    highest_score = max(highest_score,run_score)
print('highest score:', highest_score)
print('false_positive:',fp_num)
print('true_negative:',tn_num)
np.savetxt('random',run_score_list)
# plt.plot(run_score_list,'x')
# axes = plt.axes()
# axes.set_ylim([0, 100])
# plt.show()
        # for result in detect_result[0]:
        #     if result[-2] > 0.8 and result[-2] > detect_result_conf:
        #         detect_result_post = result
        #         detect_result_conf = result[-2]
        # cv2.imshow('Detection result', result_img)
        # k = cv2.waitKey(200)
        # if k == ord('q') & 0xFF:
        #     break