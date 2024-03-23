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





class Random(TestFramework):
    def __init__(self, rl_agent, pop_size, generations, mutation_rate, host_ip, scenario_topic, map_name,
                 result_save_path='/home/yao/Documents/ga_result'):
        super(Random, self).__init__(pop_size, generations, mutation_rate, host_ip, scenario_topic, map_name, result_save_path)
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

            while rospy.is_shutdown() is False:
                # print('1111111111',self.scenario_status )
                if self.scenario_status == 'end':
                    rospy.loginfo('finish scenario. save scenario result')
                    # self.generation_result.append(self.current_scenario_result)
                    self.save_scenario_result(0, current_episode)
                    time.sleep(3)
                    self.reset_scenario_data()
                    break

            current_episode += 1











