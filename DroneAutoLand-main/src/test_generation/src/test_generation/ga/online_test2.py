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


def get_state(env, scenario, npc_idx):
    marker_position = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y)
    actor_pose = env.get_pose(env.scenario_objects[npc_idx])
    actor_position = (actor_pose.position.x_val, actor_pose.position.y_val)
    drone_current_pose = env.client.simGetVehiclePose()
    drone_current_position = [drone_current_pose.position.x_val, drone_current_pose.position.y_val]

    state  = (actor_position[0]-marker_position[0],actor_position[1]-marker_position[1],
              drone_current_position[0]-marker_position[0],drone_current_position[1]-marker_position[1])

    return state


class online(TestFramework):
    def __init__(self, rl_agent, pop_size, generations, mutation_rate, host_ip, scenario_topic, map_name,
                 result_save_path='/home/yao/Documents/ga_result'):
        super(online, self).__init__(pop_size, generations, mutation_rate, host_ip, scenario_topic, map_name, result_save_path)
        self.rl_agent = rl_agent
    def run(self):
        rospy.loginfo('start Random testing')

        current_episode = 0

        while rospy.is_shutdown() is False:
            if current_episode == self.generations - 1:
                break
            rospy.loginfo("start exp:{}".format(current_episode + 1))


            initial_seed = self.initial_population()
            scenario = initial_seed[0]
            start_scenario(scenario, self.env)
            scenario_msg = scenario_to_msg(scenario)
            scenario_msg.header.stamp = rospy.Time.now()
            self.scenario_pub.publish(scenario_msg)
            print('publish initial scenario')

            while rospy.is_shutdown() is False:
                # if len(self.current_drone_traj) != 0:
                #     print(self.current_drone_traj[-1])
                # print('22222222222',self.current_drone_traj[-1][-1])
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
                    if self.scenario_status =='onlanding' or self.scenario_status =='end':
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
                if self.scenario_status =='onlanding':
                    while rospy.is_shutdown() is False:
                        if self.scenario_status == 'end':
                            break
                if self.scenario_status == 'end':
                    rospy.loginfo('finish scenario. save scenario result')
                    # self.generation_result.append(self.current_scenario_result)
                    self.save_scenario_result(0, current_episode)
                    time.sleep(3)
                    self.reset_scenario_data()
                    break

            current_episode += 1




