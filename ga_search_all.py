
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





def fitness_func_global(solution, solution_idx):
    mode = 'yolo'
    model = load_model()
    model = model.cuda()
    opt = parse_opt()
    indicator_score = 0
    global tn_num
    global fp_num
    global score_list
    global total_score
    global run_time
    fitness_value = 0
    run_score = 0

    #object search
    # weather_soluion = np.array(solution[0:11])
    # local_solution=np.array(solution[11:])

    weather_soluion = solution

    #random
    # weather_soluion = np.random.random(11)
    print(weather_soluion)

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

                if abs(obj_position_g[1] - get_obj_pose('Cube_marker').position.y_val) < 1 and abs(
                        obj_position_g[0] - get_obj_pose('Cube_marker').position.x_val) < 1 and item[-2] < 0.8:
                    tn_num+=1
                    print('true negtive detected in this image')
                    # print('Coordinate of True:',obj_position_g)
                    # cv2.imwrite(f'True_negative/tn_global_{tn_num}.jpg', image)
                    fitness_value -= item[-2].item()
                    run_score += 1





                elif abs(obj_position_g[1] - get_obj_pose('Cube_marker').position.y_val) >= 2 and abs(
                        obj_position_g[0] - get_obj_pose('Cube_marker').position.x_val) >= 2 and item[-2] > 0.8:
                    fp_num += 1
                    run_score += 1
                    fitness_value += item[-2].item()
                    print('False positive detected in this image')
                    # print('Coordinate of False:',obj_position_g)
                    # cv2.imwrite(f'False_positive/fp_global_{fp_num}.jpg', image)
        else:
            fitness_value-=1


        if get_obj_pose('Copter').position.y_val > 50:
            break
    # des_re = destroy_mat(object1)
    # if des_re:
    #     print('object is destroyed')
    run_time+=1
    print('score: ', run_score)
    print('run time:', run_time)
    print('__________________________________________________________________________')
    score_list.append(run_score)
    total_score+=run_score
    return fitness_value


if __name__ == "__main__":
    final_result = []
    for i in range(10):
        print(f'this is {i} time')
        tn_num = 0
        fp_num = 0
        score_list=[]
        total_score = 0
        run_time = 0
        # solution_global = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0])
        solution_global = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ])

        # solutions = []

        fitness_function = fitness_func_global

        num_generations = 100
        num_parents_mating = 4

        sol_per_pop = 8
        num_genes = len(solution_global)

        init_range_low = 0
        init_range_high = 1

        parent_selection_type = "sss"
        keep_parents = 1

        crossover_type = "uniform"

        mutation_type = "swap"
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
                               random_mutation_max_val=1,
                               stop_criteria=["saturate_100000000"])

        ga_instance.run()

        solution, solution_fitness, solution_idx = ga_instance.best_solution()

        print("Parameters of the best solution : {solution}".format(solution=solution))
        print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
        print('total score: ', total_score)
        ratio = total_score/run_time
        final_result.append(ratio)
        # ga_instance.plot_fitness()
        # np.savetxt('1obj',score_list)
    print('final result:',final_result)