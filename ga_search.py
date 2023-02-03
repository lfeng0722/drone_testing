import pygad
import numpy
import cv2
import os
import time
import airsim
import torch
from airsim_api import *
from yolo_detector import load_model, detect
from yolov5.detect import parse_opt
from auto_land import pixel_to_pos, coord_convert



def fitness_func(solution, solution_idx):
    # global solutions
    # print(solution)
    fitness = 0

    for c in range(5):

        restart()
        set_env_params(solution)
        # time.sleep(5)

        # solution = np.array()
        # time.sleep(5)
        # solution = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        detect_count = 0
        for i in range(60):
            drone_pose = get_obj_pose('Copter')
            drone_position = drone_pose.position
            drone_orientation = drone_pose.orientation
            image = get_current_scene()




            if mode == 'opencv':
                arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
                arucoParams = cv2.aruco.DetectorParameters_create()
                (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
                                                            parameters=arucoParams)
                image_debug = cv2.aruco.drawDetectedMarkers(image, corners, ids)
                cv2.imshow('Detection result', image_debug)
                k = cv2.waitKey(200)
                if k == ord('q') & 0xFF:
                    break
                # print(solution_idx, ids)

                fitness1 = 1 / np.linalg.norm(np.array(solution)-np.zeros(11))
                # if not ids:
                #     fitness2 = 1
                # else:
                #     fitness2 = 0
                
                
                # fitness += fitness2

                try:
                    fitness2 = 3 - len(ids)
                except :
                    fitness2 = 3

                time.sleep(0.1)
            elif mode == 'yolo':
                # fitness1 = 1 / np.linalg.norm(np.array(solution)-np.zeros(11))
                detect_result, result_img = detect(model, image, opt)
                if len(detect_result[0]) != 0:
                    for result in detect_result[0]:
                        obj_position_c = pixel_to_pos(drone_position.z_val, result)
                        print('obj_position_c: ', obj_position_c)
                        obj_position_g = coord_convert(obj_position_c, drone_position, drone_orientation=drone_orientation)
                        print('obj_position_g', obj_position_g)
                        if abs(obj_position_g[1]-8.1) <= 1:
                            fitness += 1-result[-2].item()
                            print('true positive detect conf: ', result[-2].item())
                        else:
                            false_detection_conf = result[-2].item()
                            print('false positive detect conf: ', false_detection_conf)
                            if false_detection_conf >= 0.7 and false_detection_conf <0.8:
                                fitness += false_detection_conf
                            elif false_detection_conf > 0.8:
                                fitness += 1.5*false_detection_conf
                    
                    # fitness /= len(detect_result)

                # else:
                #     fitness = 0

                cv2.imshow('Detection result', result_img)
                k = cv2.waitKey(200)
                if k == ord('q') & 0xFF:
                    break

        height = 5 + 20 * solution[6]
        fly_to(0, 0, -height)
        time.sleep(10)

    fitness /= 5

    print(fitness)


    # client.landAsync()
    # time.sleep(5)

    # time.sleep(2)
    # global exp_count
    # exp_count += 1

    # if exp_count % 30 == 0:
    #     print('found solutions: ', solutions)
    return fitness


if __name__ == "__main__":
    print(client.simGetObjectPose('Cube_marker'))
    mode = 'yolo'
    model = load_model()
    model = model.cuda()
    model.eval()
    opt = parse_opt()

    # image = cv2.imread('board.jpg')
    # function_inputs = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    solution = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # image = get_current_scene(10) 
    # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    # arucoParams = cv2.aruco.DetectorParameters_create()
    solutions = []
    num_wrong = 0
    exp_count = 0
    fitness_function = fitness_func

    num_generations = 10
    num_parents_mating = 4

    sol_per_pop = 8
    num_genes = len(solution)

    init_range_low = 0
    init_range_high = 1

    parent_selection_type = "sss"
    keep_parents = 1

    crossover_type = "single_point"

    mutation_type = "random"
    mutation_percent_genes = 10
    # fitness_func([0.76156,     0.59797,      1.4389,     0.13461,     0.60521,      1.7365,     0.99081], 0)

    ga_instance = pygad.GA(num_generations=num_generations,
                        num_parents_mating=num_parents_mating,
                        fitness_func=fitness_function,
                        sol_per_pop=sol_per_pop,
                        num_genes=num_genes,
                        init_range_low=init_range_low,
                        init_range_high=init_range_high,
                        parent_selection_type=parent_selection_type,
                        keep_parents=keep_parents,
                        crossover_type=crossover_type,
                        mutation_type=mutation_type,
                        mutation_percent_genes=mutation_percent_genes,
                        random_mutation_min_val=0,
                        random_mutation_max_val=1)

    ga_instance.run()

    solution, solution_fitness, solution_idx = ga_instance.best_solution()
    print("Parameters of the best solution : {solution}".format(solution=solution))
    print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
