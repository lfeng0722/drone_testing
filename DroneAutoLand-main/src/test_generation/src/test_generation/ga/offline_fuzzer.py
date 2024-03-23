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



def cal_reward(scenario_result):

    return scenario_result.get_result()['land_deviation'] + scenario_result.get_result()['land_duration']

def compute_distances(arr):
    n = arr.shape[0]
    distances_sum = np.zeros(n)

    for i in range(n):
        distances = np.sqrt(np.sum((arr - arr[i]) ** 2, axis=1))
        distances_sum[i] = -np.sum(distances)

    return distances_sum


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




class Offline(TestFramework):
    def __init__(self, rl_agent, pop_size, generations, mutation_rate,env, scenario_topic, map_name, result_save_path='/home/yao/Documents/diversity_ga_result'):
        super(Offline, self).__init__(pop_size, generations, mutation_rate, env, scenario_topic, map_name, result_save_path)
        self.violation_cases = []
        self.rl_agent = rl_agent

    def run(self):
        rospy.loginfo('start test generation')
        current_episode= 0
        initial_seed = self.initial_population()
        scenario = initial_seed[0]

        offline_agent = self.rl_agent(11, [m for m in range(18)])
        memory = []
        batch_size = 32
        while rospy.is_shutdown() is False:
            if current_episode == self.generations - 1:
                break
            rospy.loginfo("start exp:{}".format(current_episode + 1))
            done = False

            start_scenario(scenario, self.env)
            scenario_msg = scenario_to_msg(scenario)
            scenario_msg.header.stamp = rospy.Time.now()
            self.scenario_pub.publish(scenario_msg)
            print('publish initial scenario')
            time.sleep(5)

            while rospy.is_shutdown() is False:
                # if len(self.current_drone_traj) != 0:
                #     print(self.current_drone_traj[-1])
                if len(self.current_drone_traj) != 0 and abs(self.current_drone_traj[-1][-1]) >= 6:
                    print("NPCs start moving")
                    break
                else:
                    time.sleep(0.5)

            while rospy.is_shutdown() is False:


                if self.scenario_status == 'end':
                    # fitness_values.append(cal_fitness(self.current_scenario_result))
                    weather = scenario.weather.to_vec()
                    daytime = scenario.time.to_vec()
                    state = weather + daytime
                    action = offline_agent.get_action(state)

                    if action == 0:
                        weather[0] += 0.1
                        print('increase rain')
                    if action == 1:
                        weather[0] -= 0.1
                        print('decrease rain')
                    if action == 2:
                        weather[1] += 0.1
                        print('increase Roadwetness')
                    if action == 3:
                        weather[1] -= 0.1
                        print('decrease Roadwetness')
                    if action == 4:
                        weather[2] += 0.1
                        print('increase Snow')
                    if action == 5:
                        weather[2] -= 0.1
                        print('decrease Snow')
                    if action == 6:
                        weather[3] += 0.1
                        print('increase RoadSnow')
                    if action == 7:
                        weather[3] -= 0.1
                        print('decrease RoadSnow')
                    if action == 8:
                        weather[4] += 0.1
                        print('increase MapleLeaf')
                    if action == 9:
                        weather[4] -= 0.1
                        print('decrease MapleLeaf')
                    if action == 10:
                        weather[5] += 0.1
                        print('increase RoadLeaf')
                    if action == 11:
                        weather[5] -= 0.1
                        print('decrease RoadLeaf')
                    if action == 12:
                        weather[6] += 0.1
                        print('increase Dust')
                    if action == 13:
                        weather[6] -= 0.1
                        print('decrease Dust')
                    if action == 14:
                        weather[7] += 0.1
                        print('increase Fog')
                    if action == 15:
                        weather[7] -= 0.1
                        print('decrease Fog')
                    if action == 16:
                        daytime[0] += 0.1
                        print('increase time')
                    if action == 17:
                        daytime[1] -= 0.1
                        print('decrease time')
                    # print(scenario)
                    scenario.weather = scenario.weather.from_vec(weather)
                    scenario.time= scenario.time.from_vec(daytime)
                    next_state = weather + daytime
                    # print(scenario)

                    rospy.loginfo('finish scenario. save scenario result')
                    scenario_result = self.current_scenario_result
                    reward = cal_reward(scenario_result)

                    memory.append((state, action, reward, next_state, done))

                    new_scenario = self.initial_population()[0]
                    new_scenario.weather = scenario.weather
                    new_scenario.time = scenario.time
                    scenario = new_scenario
                    self.save_scenario_result(0, current_episode)
                    time.sleep(3)
                    self.reset_scenario_data()

                    break
                time.sleep(1)
            if current_episode % 20 == 0:
                offline_agent.update_target()
                print("agent updated")
            if len(memory) > batch_size:
                batch = random.sample(memory, batch_size)
                loss = offline_agent.train([(s, a, r, ns, d) for s, a, r, ns, d in batch])
                print("NN Updated")
            current_episode += 1


