#!/usr/bin/env python3
from typing import List, Set
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
import copy

def non_dominated_sorting_with_weights(solutions: List[List[float]],population) -> List[List[float]]:
    def dominates(sol1: List[float], sol2: List[float]) -> bool:
        """Check if sol1 dominates sol2."""
        return all(x <= y for x, y in zip(sol1, sol2)) and any(x < y for x, y in zip(sol1, sol2))

    def calculate_crowding_distance(front_solutions: List[List[float]]) -> List[float]:
        """Calculate the crowding distance of each solution in the front."""
        if not front_solutions:
            return []

        size = len(front_solutions)
        distances = [0.0 for _ in range(size)]
        for m in range(len(front_solutions[0])):
            sorted_indices = sorted(range(size), key=lambda x: front_solutions[x][m])
            distances[sorted_indices[0]] = float('inf')
            distances[sorted_indices[-1]] = float('inf')
            for i in range(1, size - 1):
                distances[sorted_indices[i]] += (
                            front_solutions[sorted_indices[i + 1]][m] - front_solutions[sorted_indices[i - 1]][m])

        return distances

    # Non-dominated sorting
    n = len(solutions)
    dominated_by = [set() for _ in range(n)]
    dominates_count = [0 for _ in range(n)]
    fronts = [[]]

    for i in range(n):
        for j in range(n):
            if i != j:
                if dominates(solutions[i], solutions[j]):
                    dominated_by[i].add(j)
                elif dominates(solutions[j], solutions[i]):
                    dominates_count[i] += 1
        if dominates_count[i] == 0:
            fronts[0].append(i)

    i = 0
    while fronts[i]:
        next_front = []
        for sol in fronts[i]:
            for dominated_sol in dominated_by[sol]:
                dominates_count[dominated_sol] -= 1
                if dominates_count[dominated_sol] == 0:
                    next_front.append(dominated_sol)
        i += 1
        fronts.append(next_front)

    fronts = [set(front) for front in fronts if front]

    # Sorting within each front based on crowding distance
    sorted_population_indices = []
    for front in fronts:
        front_list = list(front)
        front_solutions = [solutions[i] for i in front_list]
        crowding_distances = calculate_crowding_distance(front_solutions)
        # Sort the front based on crowding distance (descending order)
        sorted_front_indices = sorted(front_list, key=lambda i: crowding_distances[front_list.index(i)], reverse=True)
        sorted_population_indices.extend(sorted_front_indices)

    sorted_population = [population[i] for i in sorted_population_indices]

    return sorted_population


def non_dominated_sorting_initial(solutions: List[List[float]]) -> (List[Set[int]], List[float]):
    def dominates(sol1: List[float], sol2: List[float]) -> bool:
        """Check if sol1 dominates sol2."""
        return all(x <= y for x, y in zip(sol1, sol2)) and any(x < y for x, y in zip(sol1, sol2))

    # Non-dominated sorting
    n = len(solutions)
    dominated_by = [set() for _ in range(n)]
    dominates_count = [0 for _ in range(n)]
    fronts = [[]]

    for i in range(n):
        for j in range(n):
            if i != j:
                if dominates(solutions[i], solutions[j]):
                    dominated_by[i].add(j)
                elif dominates(solutions[j], solutions[i]):
                    dominates_count[i] += 1
        if dominates_count[i] == 0:
            fronts[0].append(i)

    i = 0
    while fronts[i]:
        next_front = []
        for sol in fronts[i]:
            for dominated_sol in dominated_by[sol]:
                dominates_count[dominated_sol] -= 1
                if dominates_count[dominated_sol] == 0:
                    next_front.append(dominated_sol)
        i += 1
        fronts.append(next_front)

    fronts = [set(front) for front in fronts if front]

    # Assigning weights
    weights = [0] * n
    for i, front in enumerate(fronts):
        weight = 1 / (i + 1)
        for sol in front:
            weights[sol] = weight

    return fronts, weights


def distance_2d(point1, point2):
    """Calculate the Euclidean distance between two points in 2D space."""
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)


def normalize(lst):
    min_val = min(lst)
    max_val = max(lst)
    
    if max_val == min_val:  # Avoid division by zero if all numbers are the same
        return [0.5 for _ in lst]
    
    return [(x - min_val) / (max_val - min_val) for x in lst]

def compute_distances(arr):
    n = arr.shape[0]
    distances_sum = np.zeros(n)

    for i in range(n):
        distances = np.sqrt(np.sum((arr - arr[i]) ** 2, axis=1))
        distances_sum[i] = -np.sum(distances)

    return distances_sum

def cal_fitness_3(scenario_result, population):
    solution = []
    weather =[]
    for result in scenario_result:
        # print('111111111111',result.scenario.weather.to_vec())
        weather.append(result.scenario.weather.to_vec())
    diverse_score = compute_distances(np.array(weather))
    for i, result in enumerate(scenario_result):
        solution.append([-result.get_result()['land_deviation'], -result.get_result()['land_duration'],diverse_score[i]])


    sorted_population = non_dominated_sorting_with_weights(solution,population)
    return sorted_population


def cal_fitness_1(scenario_result, scenario_rewards=None):
    solution = []
    weather =[]
    for result in scenario_result:
        # print('111111111111',result.scenario.weather.to_vec())
        weather.append(result.scenario.weather.to_vec())
    diverse_score = compute_distances(np.array(weather))
    for i, result in enumerate(scenario_result):
        solution.append([-result.get_result()['land_deviation'], -result.get_result()['land_duration'],diverse_score[i]])

    rank, weight = non_dominated_sorting_initial(solution)
    return rank, weight


class GA(TestFramework):
    def __init__(self, pop_size, generations, mutation_rate, host_ip, scenario_topic, map_name, result_save_path='/home/yao/Documents/ga_result'):
        super(GA, self).__init__(pop_size, generations, mutation_rate, host_ip, scenario_topic, map_name, result_save_path)
    def run(self):
        rospy.loginfo('start test generation')
        population = self.initial_population()
        # population+=population
        # print('111111111111111',len(population))
        # generation_population = copy.deepcopy(population)
        current_generation = 0
        while rospy.is_shutdown() is False:
            if current_generation == self.generations - 1:
                
                break

            current_scenario_id = 0
            while rospy.is_shutdown() is False:
                if current_scenario_id == self.pop_size:
                    rank, weight = cal_fitness_1(self.generation_result)
                    print('rank: {}'.format(rank))
                    parents = random.choices(population, weights=weight, k=self.pop_size)

                    for i in range(0, self.pop_size, 2):
                        parent_1, parent_2 = parents[i], parents[i + 1]
                        child_1, child_2 = ScenarioObj.crossover(parent_1, parent_2)
                        child_1.mutate(self.mutation_rate)
                        child_2.mutate(self.mutation_rate)
                        population.append(child_1)
                        population.append(child_2)

                elif current_scenario_id == 2*self.pop_size:
                    sorted_population = cal_fitness_3(self.generation_result,population)
                    population=sorted_population[0:self.pop_size]

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
                while rospy.is_shutdown() is False:
                    # if len(self.current_drone_traj) != 0:
                    #     print(self.current_drone_traj[-1])
                    if len(self.current_drone_traj) != 0 and abs(self.current_drone_traj[-1][-1]) >= 6:
                        print("NPCs start moving")
                        break
                    else:
                        time.sleep(0.5)
                # check whether the drone has finished taking off
                # time.sleep(5)
                self.env.run_scenario()
                while rospy.is_shutdown() is False:

                    if self.scenario_status == 'end':
                        # fitness_values.append(cal_fitness(self.current_scenario_result))
                        rospy.loginfo('finish scenario. save scenario result')
                        self.generation_result.append(self.current_scenario_result)
                        self.save_scenario_result(current_generation, current_scenario_id)
                        time.sleep(3)
                        self.reset_scenario_data()
                        break

                    time.sleep(1)
                




            







