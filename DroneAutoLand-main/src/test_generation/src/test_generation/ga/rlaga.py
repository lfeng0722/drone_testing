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
import rospy
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import CommandCode


def compute_distances(arr):
    n = arr.shape[0]
    distances_sum = np.zeros(n)

    for i in range(n):
        distances = np.sqrt(np.sum((arr - arr[i]) ** 2, axis=1))
        distances_sum[i] = -np.sum(distances)

    return distances_sum
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



def distance_2d(point1, point2):
    """Calculate the Euclidean distance between two points in 2D space."""
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)


def diversity_mutate(violation_pool, current_val, num_candiates=10, bound=(0, 1), mutation_rate=0.2):
    flag = random.random()
    if flag < mutation_rate:
        # Step 1: Compute the centroid
        centroid = sum(violation_pool) / len(violation_pool)


        # Step 2: Sample 10 random numbers
        sampled_numbers = [random.uniform(bound[0], bound[1]) for _ in range(num_candiates)]

        # Step 3 & 4: Compute the distance and save the sampled number with the largest distance to the centroid
        farthest_sample = max(sampled_numbers, key=lambda x: abs(x - centroid))

        return farthest_sample
    else:
        return current_val

def get_state(env, scenario, npc_idx):
    marker_position = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y)
    actor_pose = env.get_pose(env.scenario_objects[npc_idx])
    actor_position = (actor_pose.position.x_val, actor_pose.position.y_val)
    drone_current_pose = env.client.simGetVehiclePose()
    drone_current_position = [drone_current_pose.position.x_val, drone_current_pose.position.y_val]

    state  = (actor_position[0]-marker_position[0],actor_position[1]-marker_position[1],
              drone_current_position[0]-marker_position[0],drone_current_position[1]-marker_position[1])

    return state

def normalize(lst):
    min_val = min(lst)
    max_val = max(lst)

    if max_val == min_val:  # Avoid division by zero if all numbers are the same
        return [0.5 for _ in lst]

    return [(x - min_val) / (max_val - min_val) for x in lst]


# def cal_fitness(scenario_result, scenario_rewards=None):
#     solution = []
#
#     for result in scenario_result:
#         solution.append([result['land_deviation'],result['land_duration']])
#         # landing_times.append(result['land_duration'])
#         # landing_distance.append(result['land_deviation'] )
#         # if result['land_deviation'] < 1.5:
#         #     fitness_dist = 0.5
#         # else:
#         #     fitness_dist = 0.2 * 1 / abs(1.5-result['land_deviation'])
#         # fitness_values.append(result['land_success'] + fitness_dist)
#
#     # landing_times = normalize(landing_times)
#     # for i, landing_time in enumerate(landing_times):
#     #     fitness_values[i] += landing_time
#     rank, weight = non_dominated_sorting_with_weights(solution)
#     return rank, weight


class RLAGA(TestFramework):
    def __init__(self, rl_agent, pop_size, generations, mutation_rate,env, scenario_topic, map_name, result_save_path='/home/yao/Documents/diversity_ga_result'):
        super(RLAGA, self).__init__(pop_size, generations, mutation_rate, env, scenario_topic, map_name, result_save_path)
        self.violation_cases = []
        self.rl_agent = rl_agent

    def run(self):
        rospy.loginfo('start test generation')
        population = self.initial_population()
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
                        child_1 = self.mutate(child_1)
                        child_2 = self.mutate(child_2)
                        # child_1.mutate(self.mutation_rate)
                        # child_2.mutate(self.mutation_rate)
                        population.append(child_1)
                        population.append(child_2)

                elif current_scenario_id == 2 * self.pop_size:
                    sorted_population = cal_fitness_3(self.generation_result, population)
                    population = sorted_population[0:self.pop_size]

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

                while rospy.is_shutdown() is False:
                    # if len(self.current_drone_traj) != 0:
                    #     print(self.current_drone_traj[-1])
                    if len(self.current_drone_traj) != 0 and abs(self.current_drone_traj[-1][-1]) >= 6:
                        print("NPCs start moving")
                        break
                    else:
                        time.sleep(0.5)

                agents = [self.rl_agent(4,[0, 1, 2, 3, 4]) for _ in range(len(self.env.scenario_objects))]
                # self.env.run_scenario()
                while rospy.is_shutdown() is False:
                    idx = 0
                    while rospy.is_shutdown() is False:
                        if idx == len(self.env.scenario_objects):
                            break
                        if self.scenario_status == 'onlanding' or self.scenario_status == 'end':
                            # print('1111111',self.env.scenario_objects)
                            for obj in self.env.scenario_objects:
                                self.env.set_npc_forward(obj, 5)
                                # self.env.move_npc_forward(obj, 5)
                                time.sleep(0.3)
                            # time.sleep(5)
                            break

                        actor = self.env.scenario_objects[idx]
                        agent_state = get_state(self.env, scenario, idx)
                        action = agents[idx].evaluate(agent_state)
                        print(actor, action)
                        if action == 1:
                            self.env.move_npc_forward(actor, 1)
                        elif action == 2:
                            self.env.move_npc_backward(actor, 1)
                        elif action == 3:
                            self.env.move_npc_left(actor, 1)
                        elif action == 4:
                            self.env.move_npc_right(actor, 1)

                        idx += 1
                        time.sleep(0.5)
                    if self.scenario_status == 'onlanding':
                        while rospy.is_shutdown() is False:
                            if self.scenario_status == 'end':
                                break

                    if self.scenario_status == 'end':
                        # fitness_values.append(cal_fitness(self.current_scenario_result))
                        rospy.loginfo('finish scenario. save scenario result')
                        scenario_result = self.current_scenario_result
                        if scenario_result.get_result()['land_success'] == 0:
                            self.violation_cases.append(scenario)
                        self.generation_result.append(scenario_result)

                        self.save_scenario_result(current_generation, current_scenario_id)
                        time.sleep(3)
                        self.reset_scenario_data()
                        break

                    time.sleep(1)


    def mutate(self, scenario):
        # tp_marker
        if len(self.violation_cases) == 0:
            scenario.mutate()
        else:
            tp_marker = scenario.tp_marker
            violation_x = [violation.tp_marker.pose.x for violation in self.violation_cases]
            violation_y = [violation.tp_marker.pose.y for violation in self.violation_cases]

            tp_marker.pose.x = diversity_mutate(violation_x, tp_marker.pose.x, bound=self.env.map_config["random_pos_x"], mutation_rate=self.mutation_rate)
            tp_marker.pose.y = diversity_mutate(violation_y, tp_marker.pose.y, bound=self.env.map_config["random_pos_y"], mutation_rate=self.mutation_rate)

            # gps_pose
            gps_pose = scenario.gps_pose
            gps_pose.x = tp_marker.pose.x + random.uniform(-5, 5)
            gps_pose.y = tp_marker.pose.y + random.uniform(-5, 5)

            # fp_markers
            for i, marker in enumerate(scenario.fp_markers):
                violation_x = [violation.fp_markers[i].pose.x for violation in self.violation_cases]
                violation_y = [violation.fp_markers[i].pose.y for violation in self.violation_cases]
                marker.pose.x = diversity_mutate(violation_x, marker.pose.x, bound=self.env.map_config["random_pos_x"], mutation_rate=self.mutation_rate)
                marker.pose.y = diversity_mutate(violation_y, marker.pose.y, bound=self.env.map_config["random_pos_y"], mutation_rate=self.mutation_rate)

            # drone_start_pose
            drone_start_pose = scenario.drone_start_pose
            violation_x = [violation.drone_start_pose.x for violation in self.violation_cases]
            violation_y = [violation.drone_start_pose.y for violation in self.violation_cases]
            drone_start_pose.x = diversity_mutate(violation_x, drone_start_pose.x,
                                                  bound=self.env.map_config["random_pos_x"], mutation_rate=self.mutation_rate)
            drone_start_pose.y = diversity_mutate(violation_y, drone_start_pose.y,
                                                  bound=self.env.map_config["random_pos_y"], mutation_rate=self.mutation_rate)


            # weather
            weather = scenario.weather
            violation_rain = [violation.weather.rain for violation in self.violation_cases]
            weather.rain = diversity_mutate(violation_rain, weather.rain, bound=(0, 0.15),
                                            mutation_rate=self.mutation_rate)
            violation_road_wetness = [violation.weather.road_wetness for violation in self.violation_cases]
            weather.road_wetness = diversity_mutate(violation_road_wetness, weather.road_wetness, bound=(0, 0.15),
                                                    mutation_rate=self.mutation_rate)
            violation_rain = [violation.weather.rain for violation in self.violation_cases]
            weather.rain = diversity_mutate(violation_rain, weather.rain, bound=(0, 0.15),
                                            mutation_rate=self.mutation_rate)
            violation_road_snow = [violation.weather.road_snow for violation in self.violation_cases]
            weather.road_snow = diversity_mutate(violation_road_snow, weather.road_snow, bound=(0, 0.15),
                                                 mutation_rate=self.mutation_rate)
            violation_maple_leaf = [violation.weather.maple_leaf for violation in self.violation_cases]
            weather.maple_leaf = diversity_mutate(violation_maple_leaf, weather.maple_leaf, bound=(0, 0.15),
                                                  mutation_rate=self.mutation_rate)
            violation_road_leaf = [violation.weather.road_leaf for violation in self.violation_cases]
            weather.road_leaf = diversity_mutate(violation_road_leaf, weather.road_leaf, bound=(0, 0.15),
                                                 mutation_rate=self.mutation_rate)
            violation_dust = [violation.weather.dust for violation in self.violation_cases]
            weather.dust = diversity_mutate(violation_dust, weather.dust, bound=(0, 0.15),
                                            mutation_rate=self.mutation_rate)
            violation_fog = [violation.weather.fog for violation in self.violation_cases]
            weather.fog = diversity_mutate(violation_fog, weather.fog, bound=(0, 0.15),
                                           mutation_rate=self.mutation_rate)
            violation_wind = [violation.weather.wind for violation in self.violation_cases]
            weather.wind = diversity_mutate(violation_wind, weather.wind, bound=(0, 0.15),
                                            mutation_rate=self.mutation_rate)

            # time
            time = scenario.time
            violation_hour = [violation.time.hour for violation in self.violation_cases]
            time.hour = diversity_mutate(violation_hour, time.hour, bound=(0, 1),
                                         mutation_rate=self.mutation_rate)
            violation_minute = [violation.time.minute for violation in self.violation_cases]
            time.minute = diversity_mutate(violation_minute, time.minute, bound=(0, 1),
                                           mutation_rate=self.mutation_rate)

            # actor
            violation_actors = {}
            for violation in self.violation_cases:
                for actor in violation.actors:
                    if actor.type not in violation_actors.keys():
                        violation_actors[actor.type] = [actor]
                    else:
                        violation_actors[actor.type].append(actor)

            for actor in scenario.actors:
                try:
                    violation_actor = violation_actors[actor.type]
                    violation_actor_sx = [v_actor.start_pose.x for v_actor in violation_actor]
                    violation_actor_sy = [v_actor.start_pose.y for v_actor in violation_actor]
                    violation_actor_ex = [v_actor.end_pose.x for v_actor in violation_actor]
                    violation_actor_ey = [v_actor.end_pose.y for v_actor in violation_actor]
                    violation_actor_speed = [v_actor.speed for v_actor in violation_actor]
                    actor.start_pose.x = diversity_mutate(violation_actor_sx, actor.start_pose.x,
                                                          bound=self.env.map_config["random_pos_x"], mutation_rate=self.mutation_rate)
                    actor.start_pose.y = diversity_mutate(violation_actor_sy, actor.start_pose.y,
                                                          bound=self.env.map_config["random_pos_y"], mutation_rate=self.mutation_rate)
                    actor.end_pose.x = diversity_mutate(violation_actor_ex, actor.end_pose.x,
                                                        bound=self.env.map_config["random_pos_x"], mutation_rate=self.mutation_rate)
                    actor.end_pose.y = diversity_mutate(violation_actor_ey, actor.end_pose.y,
                                                        bound=self.env.map_config["random_pos_y"], mutation_rate=self.mutation_rate)
                    actor.speed = diversity_mutate(violation_actor_speed, actor.speed,
                                                   bound=(0, 1), mutation_rate=self.mutation_rate)
                except:
                    actor.mutate()

        return scenario











