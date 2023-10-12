#!/usr/bin/env python3

import random
import numpy as np
import rospy
from test_generation.ga.test_framework import TestFramework, start_scenario, scenario_to_msg
from airsim_simulation.scenario import sample_scenario, sample_test_scenario
from nav_msgs.msg import Path
import time
from geometry_msgs.msg import PoseStamped
from airsim_simulation.configuration import ACTOR_TYPE_DICT
from simulation.msg import Scenario, Marker, Actor, ScenarioPose, TimeOfDay, Weather
from airsim_simulation.scenario import Scenario as ScenarioObj
from airsim_simulation.components import Pose
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pickle
import os
from airsim_simulation.sim_env import AirSimEnv




def distance_2d(point1, point2):
    """Calculate the Euclidean distance between two points in 2D space."""
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)



def normalize(lst):
    min_val = min(lst)
    max_val = max(lst)
    
    if max_val == min_val:  # Avoid division by zero if all numbers are the same
        return [0.5 for _ in lst]
    
    return [(x - min_val) / (max_val - min_val) for x in lst]


def cal_fitness(scenario_result, scenario_rewards=None):
    fitness_values = []
    landing_times = []
    for result in scenario_result:
        landing_times.append(result['land_duration'])
        fitness_values.append(result['land_success'])

    landing_times = normalize(landing_times)
    for i, landing_time in enumerate(landing_times):
        fitness_values[i] += landing_time
    
    return fitness_values
    

class GA(TestFramework):
    def __init__(self, pop_size, generations, mutation_rate, host_ip, scenario_topic, result_save_path='/home/yao/Documents/ga_result'):
        super(GA, self).__init__(pop_size, generations, mutation_rate, host_ip, scenario_topic, result_save_path)






    def run(self):
        rospy.loginfo('start test generation')
        population = self.initial_population()
        current_generation = 0
        while rospy.is_shutdown() is False:
            if current_generation == self.generations - 1:
                
                break

            current_scenario_id = 0
            while rospy.is_shutdown() is False:
                if current_scenario_id == len(population):
                    fitness_values = cal_fitness(self.generation_result)
                    print('fitness values: {}'.format(fitness_values))
                    parents = random.choices(population, weights=fitness_values, k=self.pop_size)
                    next_generation = []
                    for i in range(0, self.pop_size, 2):
                        parent_1, parent_2 = parents[i], parents[i + 1]
                        child_1, child_2 = ScenarioObj.crossover(parent_1, parent_2)
                        child_1.mutate(self.mutation_rate)
                        child_2.mutate(self.mutation_rate)
                        next_generation.append(child_1)
                        next_generation.append(child_2)
                
                    population = next_generation
                    current_generation += 1
                    self.generation_result = []
                    
                    break

                rospy.loginfo("start exp:{}-{}".format(current_generation+1, current_scenario_id+1))
                scenario = population[current_scenario_id]
                current_scenario_id += 1

                start_scenario(scenario, self.env)
                scenario_msg = scenario_to_msg(scenario)
                # scenario_msg = scenario.to_msg()
                scenario_msg.header.stamp = rospy.Time.now()

                self.scenario_pub.publish(scenario_msg)
                print('publish scenario')
                time.sleep(5)
                self.env.run_scenario()
                while rospy.is_shutdown() is False:

                    if self.scenario_status == 'end':
                        # fitness_values.append(cal_fitness(self.current_scenario_result))
                        rospy.loginfo('finish scenario. save scenario result')
                        self.generation_result.append(self.current_scenario_result.get_result())
                        self.save_scenario_result(current_generation, current_scenario_id)
                        time.sleep(3)
                        self.reset_scenario_data()
                        break
                
                # while rospy.is_shutdown() is False:   
                    # drone_pose = self.env.client.simGetVehiclePose()
                    # if abs(drone_pose.position.x_val) < 0.3 and abs(drone_pose.position.y_val) < 0.3 and \
                    #     self.scenario_status == 'end':
                        # time.sleep(3)
                        # print('4444444444444444444444')
                        # break

                    # rate.sleep()
                    time.sleep(1)
                




            







