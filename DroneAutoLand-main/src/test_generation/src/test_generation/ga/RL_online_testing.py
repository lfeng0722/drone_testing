#!/usr/bin/env python3
import copy
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


def non_dominated_sorting_with_weights(solutions: List[List[float]]) -> (List[Set[int]], List[float]):
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
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)


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


def cal_fitness_3(scenario_result, scenario_rewards=None):
    solution = []
    weather = []
    for result in scenario_result:
        # print('111111111111',result.scenario.weather.to_vec())
        weather.append(result.scenario.weather.to_vec())
    diverse_score = compute_distances(np.array(weather))
    for i, result in enumerate(scenario_result):
        solution.append(
            [-result.get_result()['land_deviation'], -result.get_result()['land_duration'], diverse_score[i]])

    rank, weight = non_dominated_sorting_with_weights(solution)
    return rank, weight

def get_state(env, scenario):
    marker_position = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y)

    drone_current_pose = env.client.simGetVehiclePose()
    drone_current_position = [drone_current_pose.position.x_val, drone_current_pose.position.y_val]

    state  = (marker_position[0],marker_position[1],
              drone_current_position[0],drone_current_position[1])

    return state

class RL_online(TestFramework):
    def __init__(self, rl_agent, pop_size, generations, mutation_rate, host_ip, scenario_topic,
                 map_name, result_save_path='/home/yao/Documents/ga_result'):
        super(RL_online, self).__init__(pop_size, generations, mutation_rate, host_ip, scenario_topic, map_name, result_save_path)
        self.rl_agent = rl_agent
    def run(self):
        rospy.loginfo('start RL online testing')

        current_episode = 0



        agent = self.rl_agent(15, [m for m in range(23)])
        while rospy.is_shutdown() is False:
            if current_episode == self.generations - 1:
                break
            rospy.loginfo("start exp:{}".format(current_episode + 1))
            # reward = 0
            # done = False

            initial_seed = self.initial_population()
            scenario = initial_seed[0]
            start_scenario(scenario, self.env)
            weather = scenario.weather.to_vec()
            daytime = scenario.time.to_vec()
            relative_position = get_state(self.env, scenario)
            state = weather + daytime + list(relative_position)
            scenario_msg = scenario_to_msg(scenario)
            scenario_msg.header.stamp = rospy.Time.now()
            self.scenario_pub.publish(scenario_msg)
            print('publish initial scenario')

            while rospy.is_shutdown() is False:
                # if len(self.current_drone_traj) != 0:
                #     print(self.current_drone_traj[-1])
                if len(self.current_drone_traj) != 0 and abs(self.current_drone_traj[-1][-1]) >= 6:
                    print("NPCs start moving")
                    break
                else:
                    time.sleep(0.5)

            while rospy.is_shutdown() is False:

                # print('1111111111111',len(state))

                action = agent.evaluate(state)
                # print('2222222222222', action)

                time.sleep(1)
                if action == 1 and self.scenario_status != 'end'and self.scenario_status !='onlanding':
                    if len(self.env.scenario_objects)!= 0:
                        idx = random.randint(0, len(self.env.scenario_objects)-1)
                        # print('idxx', idx)
                        # print('111111111111', len(self.env.dynamic_objects))
                        self.env.move_npc_forward(self.env.scenario_objects[idx], 1)
                        print('move forward' )

                    else:
                        pass
                if action == 2 and self.scenario_status != 'end'and self.scenario_status !='onlanding':
                    if len(self.env.scenario_objects)!= 0:
                        idx = random.randint(0, len(self.env.scenario_objects)-1)
                        # print('idxx', idx)
                        # print('111111111111', len(self.env.dynamic_objects))
                        self.env.move_npc_backward(self.env.scenario_objects[idx], 1)
                        print('move backward')

                    else:
                        pass
                if action == 3 and self.scenario_status != 'end'and self.scenario_status !='onlanding':
                    if len(self.env.scenario_objects)!= 0:
                        idx = random.randint(0, len(self.env.scenario_objects)-1)
                        # print('idxx', idx)
                        # print('111111111111', len(self.env.dynamic_objects))
                        self.env.move_npc_left(self.env.scenario_objects[idx], 1)

                        print('move left')
                    else:
                        pass
                if action == 4 and self.scenario_status != 'end'and self.scenario_status !='onlanding':
                    if len(self.env.scenario_objects)!= 0:
                        idx = random.randint(0, len(self.env.scenario_objects)-1)
                        # print('idxx', idx)
                        # print('111111111111', len(self.env.dynamic_objects))
                        self.env.move_npc_right(self.env.scenario_objects[idx], 1)
                        print('move right')

                    else:
                        pass
                if action == 5 and self.scenario_status != 'end':
                    weather[0] += 0.01
                    print('increase rain')
                if action == 6 and self.scenario_status != 'end':
                    weather[0] -= 0.01
                    print('decrease rain')
                if action == 7 and self.scenario_status != 'end':
                    weather[1] += 0.01
                    print('increase Roadwetness')
                if action == 8 and self.scenario_status != 'end':
                    weather[1] -= 0.01
                    print('decrease Roadwetness')
                if action == 9 and self.scenario_status != 'end':
                    weather[2] += 0.01
                    print('increase Snow')
                if action == 10 and self.scenario_status != 'end':
                    weather[2] -= 0.01
                    print('decrease Snow')
                if action == 11 and self.scenario_status != 'end':
                    weather[3] += 0.01
                    print('increase RoadSnow')
                if action == 12 and self.scenario_status != 'end':
                    weather[3] -= 0.01
                    print('decrease RoadSnow')
                if action == 13 and self.scenario_status != 'end':
                    weather[4] += 0.01
                    print('increase MapleLeaf')
                if action == 14 and self.scenario_status != 'end':
                    weather[4] -= 0.01
                    print('decrease MapleLeaf')
                if action == 15 and self.scenario_status != 'end':
                    weather[5] += 0.01
                    print('increase RoadLeaf')
                if action == 16 and self.scenario_status != 'end':
                    weather[5] -= 0.01
                    print('decrease RoadLeaf')
                if action == 17 and self.scenario_status != 'end':
                    weather[6] += 0.01
                    print('increase Dust')
                if action == 18 and self.scenario_status != 'end':
                    weather[6] -= 0.01
                    print('decrease Dust')
                if action == 19 and self.scenario_status != 'end':
                    weather[7] += 0.01
                    print('increase Fog')
                if action == 20 and self.scenario_status != 'end':
                    weather[7] -= 0.01
                    print('decrease Fog')
                if action == 21 and self.scenario_status != 'end':
                    daytime[0] += 0.01
                    print('increase time')
                if action == 22 and self.scenario_status != 'end':
                    daytime[1] -= 0.01
                    print('decrease time')


                self.env.set_weather(weather)
                self.env.set_time_of_day(daytime)
                # start_scenario(scenario, self.env)

                relative_position = get_state(self.env, scenario)
                next_state = weather+daytime+list(relative_position)
                #
                # if self.env.get_collision_info() in self.env.scenario_objects:
                #     reward += 1
                #
                #     print('collided')
                #
                # if self.scenario_status == 'end':
                #     done =True
                #
                # memory.append((state, action, reward, next_state, done))
                #
                state = next_state


                if self.scenario_status == 'end':
                    rospy.loginfo('finish scenario. save scenario result')
                    scenario.weather = scenario.weather.from_vec(weather)
                    scenario.time = scenario.time.from_vec(daytime)
                    self.save_scenario_result(0, current_episode)
                    time.sleep(3)
                    self.reset_scenario_data()
                    break

            # if current_episode % 20 == 0:
            #     agent.update_target()
            #     print("agent updated")
            # if len(memory) > batch_size:
            #     batch = random.sample(memory, batch_size)
            #     loss = agent.train([(s, a, r, ns, d) for s, a, r, ns, d in batch])
            #     print("NN Updated")
            current_episode += 1











