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
        if result['land_deviation'] < 1.5:
            fitness_dist = 0.5
        else:
            fitness_dist = 0.2 * 1 / abs(1.5-result['land_deviation'])
        fitness_values.append(result['land_success'] + fitness_dist)

    landing_times = normalize(landing_times)
    for i, landing_time in enumerate(landing_times):
        fitness_values[i] += landing_time
    
    return fitness_values
    

class DiversityGA(TestFramework):
    def __init__(self, pop_size, generations, mutation_rate, host_ip, scenario_topic, result_save_path):
        super(DiversityGA, self).__init__(pop_size, generations, mutation_rate, host_ip, scenario_topic, result_save_path)
        self.violation_cases = []





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
                        child_1 = self.mutate(child_1)
                        child_2 = self.mutate(child_2)
                        # child_1.mutate(self.mutation_rate)
                        # child_2.mutate(self.mutation_rate)
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
                        scenario_result = self.current_scenario_result.get_result()
                        if scenario_result['land_success'] == 0:
                            self.violation_cases.append(scenario)
                        self.generation_result.append(scenario_result)
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
                
    def mutate(self, scenario):
        # tp_marker
        if len(self.violation_cases) == 0:
            scenario.mutate()
        else:
            tp_marker = scenario.tp_marker
            violation_x = [violation.tp_marker.pose.x for violation in self.violation_cases]
            violation_y = [violation.tp_marker.pose.y for violation in self.violation_cases]

            tp_marker.pose.x = diversity_mutate(violation_x, tp_marker.pose.x, bound=(-20, 20), mutation_rate=self.mutation_rate)
            tp_marker.pose.y = diversity_mutate(violation_y, tp_marker.pose.y, bound=(-20, 20), mutation_rate=self.mutation_rate)

            # gps_pose
            gps_pose = scenario.gps_pose
            gps_pose.x = tp_marker.pose.x + random.uniform(-5, 5)
            gps_pose.y = tp_marker.pose.y + random.uniform(-5, 5)

            # fp_markers
            for i, marker in enumerate(scenario.fp_markers):
                violation_x = [violation.fp_markers[i].pose.x for violation in self.violation_cases]
                violation_y = [violation.fp_markers[i].pose.y for violation in self.violation_cases]            
                marker.pose.x = diversity_mutate(violation_x, marker.pose.x, bound=(-20, 20), mutation_rate=self.mutation_rate)
                marker.pose.y = diversity_mutate(violation_y, marker.pose.y, bound=(-20, 20), mutation_rate=self.mutation_rate)
            
            # drone_start_pose
            drone_start_pose = scenario.drone_start_pose
            violation_x = [violation.drone_start_pose.x for violation in self.violation_cases]
            violation_y = [violation.drone_start_pose.y for violation in self.violation_cases]
            drone_start_pose.x = diversity_mutate(violation_x, drone_start_pose.x,
                                                    bound=(-20, 20), mutation_rate=self.mutation_rate)
            drone_start_pose.y = diversity_mutate(violation_y, drone_start_pose.y,
                                                    bound=(-20, 20), mutation_rate=self.mutation_rate)


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
                                            bound=(-20, 20), mutation_rate=self.mutation_rate)
                    actor.start_pose.y = diversity_mutate(violation_actor_sy, actor.start_pose.y,
                                            bound=(-20, 20), mutation_rate=self.mutation_rate)
                    actor.end_pose.x = diversity_mutate(violation_actor_ex, actor.end_pose.x,
                                            bound=(-20, 20), mutation_rate=self.mutation_rate)
                    actor.end_pose.y = diversity_mutate(violation_actor_ey, actor.end_pose.y,
                                            bound=(-20, 20), mutation_rate=self.mutation_rate)
                    actor.speed = diversity_mutate(violation_actor_speed, actor.speed,
                                            bound=(0, 1), mutation_rate=self.mutation_rate)  
                except:
                    actor.mutate()

        return scenario
        


            







