import time

from airsim_api import *
from yolo_detector import load_model, detect
from yolov5.detect import parse_opt
import numpy as np
import random
from auto_land import pixel_to_pos, coord_convert
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')
import pygad


def ga_instan(fitness_func):
    solution_local = np.array([0.0, 0.0])
    fitness_function = fitness_func

    num_generations = 10
    num_parents_mating = 2

    sol_per_pop = 8
    num_genes = len(solution_local)

    init_range_low = -1
    init_range_high = 1

    parent_selection_type = "sss"
    keep_parents = 1

    crossover_type = "single_point"

    mutation_type = "random"
    mutation_percent_genes = 100

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
    return ga_instance


def local_search(run_score, obj_position_g, type):

    def fitness_fucntion(solution, solution_idx):
        global tn_num
        global fp_num
        mode = 'yolo'
        model = load_model()
        model = model.cuda()
        opt = parse_opt()
        score = run_score
        x = solution[0]
        y = solution[1]

        object1 = respawn_mat(location=obj_position_g, mat_name='Autumn_Leaves', call_name='leaf', x=x, y=y)

        print(f'{object1} has created')

        time.sleep(5)
        image_local_search = get_current_scene(0)
        detect_result_local, result_img_local = detect(model, image_local_search, opt)

        if len(detect_result_local[0]) != 0:

            for item_local in detect_result_local[0]:
                obj_position_c_local = pixel_to_pos(get_obj_pose('Copter').position.z_val, item_local)
                obj_position_g_local = coord_convert(obj_position_c_local, get_obj_pose('Copter').position,
                                                     drone_orientation=get_obj_pose('Copter').orientation)
                if abs(obj_position_g_local[1] - get_obj_pose('Cube_marker').position.y_val) <= 1 and abs(
                        obj_position_g_local[0] - get_obj_pose('Cube_marker').position.x_val) <= 1 and item_local[
                    -2] < 0.8:
                    tn_num+= 1
                    print(f'After local search, True_negative example found')
                    cv2.imwrite(f'True_negative/tn_local_{tn_num}.jpg', image_local_search)
                    score += item_local[-2].item()
                    break
                elif abs(obj_position_g_local[1] - get_obj_pose('Cube_marker').position.y_val) >1 and abs(
                        obj_position_g_local[0] - get_obj_pose('Cube_marker').position.x_val) > 1 and item_local[
                    -2] > 0.8:
                    fp_num += 1
                    print(f'After local search, False_positive example found')
                    cv2.imwrite(f'False_positive/fp_local_{fp_num}.jpg', image_local_search)
                    score += item_local[-2].item()
                    break
                else:
                    print('not found in this time')

        else:
            print(f'{type} disapeared')

        print('Local search end, destory generated objects')
        des_re = destroy_mat(object1)
        if des_re:
            print('object is destroyed')
            time.sleep(3)
        print('local score: ', score)
        print("_________________________________________________________________________________")
        return score

    return fitness_fucntion


def fitness_func_global(solution, solution_idx):
    mode = 'yolo'
    model = load_model()
    model = model.cuda()
    opt = parse_opt()
    global tn_num
    global fp_num
    # highest_score =-1
    # search_time =1000
    # run_score_list =[]
    run_score = 0

    set_current_weather(solution)
    drone_pose = respawn_drone()
    take_off(drone_pose, height=20)

    fly_to(0, 50, -20)
    print('going to destination')
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
                # print(obj_position_g[1] - get_obj_pose('Cube_marker').position.y_val)
                # print(obj_position_g[0] - get_obj_pose('Cube_marker').position.x_val)
                # TN
                if abs(obj_position_g[1] - get_obj_pose('Cube_marker').position.y_val) < 1 and abs(
                        obj_position_g[0] - get_obj_pose('Cube_marker').position.x_val) < 1 and item[-2] < 0.8:
                    tn_num+=1
                    print('true negtive detected in this image')
                    # print('Coordinate of True:',obj_position_g)
                    cv2.imwrite(f'True_negative/tn_global_{tn_num}.jpg', image)
                    run_score += item[-2].item()


                elif abs(obj_position_g[1] - get_obj_pose('Cube_marker').position.y_val) < 1 and abs(
                        obj_position_g[0] - get_obj_pose('Cube_marker').position.x_val) < 1 and item[-2] >= 0.8 and item[-2] < 0.9:

                    print('potential true negtive detected in this image, local search start')
                    fly_to(obj_position_g[0], obj_position_g[1], -20)
                    time.sleep(10)
                    type = 'True_negative'
                    local_score = 0
                    # build fitness function
                    local_fit = local_search(local_score, obj_position_g, type)

                    # building ga instance
                    ga_ins = ga_instan(local_fit)
                    ga_ins.run()
                    solution, solution_fitness, solution_idx = ga_ins.best_solution()
                    run_score = solution_fitness

                    fly_to(0, 50, -20)


                elif abs(obj_position_g[1] - get_obj_pose('Cube_marker').position.y_val) >= 2 and abs(
                        obj_position_g[0] - get_obj_pose('Cube_marker').position.x_val) >= 2 and item[-2] > 0.8:
                    fp_num += 1
                    run_score += item[-2].item()
                    print('False positive detected in this image')
                    # print('Coordinate of False:',obj_position_g)
                    cv2.imwrite(f'False_positive/fp_global_{fp_num}.jpg', image)
                elif abs(obj_position_g[1] - get_obj_pose('Cube_marker').position.y_val) >= 2 and abs(
                        obj_position_g[0] - get_obj_pose('Cube_marker').position.x_val) >= 2 and 0.8 - item[-2] < 0.1:
                    print('potential False Positive detected in this image, local search start')

                    fly_to(obj_position_g[0], obj_position_g[1], -20)
                    time.sleep(10)

                    type = 'False_positive'
                    local_score = 0
                    # build fitness function
                    local_fit = local_search(local_score, obj_position_g, type)

                    # building ga instance
                    ga_ins = ga_instan(local_fit)
                    ga_ins.run()
                    solution, solution_fitness, solution_idx = ga_ins.best_solution()
                    run_score = solution_fitness

                    fly_to(0, 50, -20)

        if get_obj_pose('Copter').position.y_val > 50:
            break
    print('score: ', run_score)
    print('__________________________________________________________________________')


    return run_score


if __name__ == "__main__":
    tn_num = 0
    fp_num = 0


    solution_global = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    solutions = []
    num_wrong = 0
    exp_count = 0
    fitness_function = fitness_func_global

    num_generations = 100
    num_parents_mating = 4

    sol_per_pop = 8
    num_genes = len(solution_global)

    init_range_low = 0
    init_range_high = 1

    parent_selection_type = "sss"
    keep_parents = 1

    crossover_type = "single_point"

    mutation_type = "random"
    mutation_percent_genes = 80
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
