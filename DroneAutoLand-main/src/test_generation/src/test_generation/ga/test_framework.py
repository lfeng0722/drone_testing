#!/usr/bin/env python3

import random
import numpy as np
import rospy
import cv2
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
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension

def scenario_to_msg(scenario_obj):
    scenario_msg = Scenario()
    tp_marker_msg = Marker()
    fp_markers_msg = []
    drone_start_pose_msg = ScenarioPose()
    gps_pose_msg = ScenarioPose()
    time_msg = TimeOfDay()
    weather_msg = Weather()
    actors_msg = []
    status = 1
    radius = scenario_obj.radius

    # tp marker
    tp_marker_msg.id = scenario_obj.tp_marker.id
    tp_marker_msg.material = scenario_obj.tp_marker.material
    tp_marker_msg.pos_x = scenario_obj.tp_marker.pose.x
    tp_marker_msg.pos_y = scenario_obj.tp_marker.pose.y
    tp_marker_msg.pos_z = scenario_obj.tp_marker.pose.z
    tp_marker_msg.yaw_angle = scenario_obj.tp_marker.pose.angle

    # fp markers
    for fp_marker in scenario_obj.fp_markers:
        fp_marker_msg = Marker()
        fp_marker_msg.id = fp_marker.id
        fp_marker_msg.material = fp_marker.material
        fp_marker_msg.pos_x = fp_marker.pose.x
        fp_marker_msg.pos_y = fp_marker.pose.y
        fp_marker_msg.pos_z = fp_marker.pose.z
        fp_marker_msg.yaw_angle = fp_marker.pose.angle
        fp_markers_msg.append(fp_marker_msg)

    # drone start pose
    drone_start_pose_msg.x = scenario_obj.drone_start_pose.x
    drone_start_pose_msg.y = scenario_obj.drone_start_pose.y
    drone_start_pose_msg.z = scenario_obj.drone_start_pose.z
    drone_start_pose_msg.yaw = scenario_obj.drone_start_pose.angle

    # gps pose
    gps_pose_msg.x = scenario_obj.gps_pose.x
    gps_pose_msg.y = scenario_obj.gps_pose.y
    gps_pose_msg.z = scenario_obj.gps_pose.z
    gps_pose_msg.yaw = scenario_obj.gps_pose.angle

    # time of day
    time_msg.hour = scenario_obj.time.hour
    time_msg.minute = scenario_obj.time.minute

    # actors
    for actor in scenario_obj.actors:
        if actor.type != -1:
            actor_msg = Actor()
            # actor_msg.id = actor.id
            actor_msg.type = ACTOR_TYPE_DICT[actor.type]
            actor_msg.start_pose.x = actor.start_pose.x
            actor_msg.start_pose.y = actor.start_pose.y
            actor_msg.start_pose.z = actor.start_pose.z
            actor_msg.start_pose.yaw = actor.start_pose.angle
            actor_msg.end_pose.x = actor.end_pose.x
            actor_msg.end_pose.y = actor.end_pose.y
            actor_msg.end_pose.z = actor.end_pose.z
            actor_msg.end_pose.yaw = actor.end_pose.angle
            actor_msg.speed = actor.speed
            actors_msg.append(actor_msg)
    
    # Weather
    weather_msg.rain = scenario_obj.weather.rain
    weather_msg.road_wetness = scenario_obj.weather.road_wetness
    weather_msg.snow = scenario_obj.weather.snow
    weather_msg.road_snow = scenario_obj.weather.road_snow
    weather_msg.maple_leaf = scenario_obj.weather.maple_leaf
    weather_msg.road_leaf = scenario_obj.weather.road_leaf
    weather_msg.dust = scenario_obj.weather.dust
    weather_msg.fog = scenario_obj.weather.fog
    weather_msg.wind = scenario_obj.weather.wind


    scenario_msg.tp_marker = tp_marker_msg
    scenario_msg.fp_markers = fp_markers_msg
    scenario_msg.drone_start_pose = drone_start_pose_msg
    scenario_msg.gps_pose = gps_pose_msg
    scenario_msg.time = time_msg
    scenario_msg.actors = actors_msg
    scenario_msg.weather = weather_msg
    scenario_msg.status = status
    scenario_msg.radius = radius
    scenario_msg.header.stamp = rospy.Time.now()

    return scenario_msg

def distance_2d(point1, point2):
    """Calculate the Euclidean distance between two points in 2D space."""
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)


class ScenarioResult:
    def __init__(self, scenario):
        self.scenario = scenario

        self.detected_landing_point =None
        self.land_duration = None
        self.land_deviation = None
        self.land_success = None
        self.land_fp_dist = None
        self.scenes = None
        self.start_land_time = None
        self.end_land_time = None
        self.land_position = None
        self.collision = 0
        self.fitness_value = 0
    
    def set_start_land_time(self, land_time):
        self.start_land_time = land_time
    
    def set_end_land_time(self, land_time):
        self.end_land_time = land_time
    
    def set_land_position(self, land_position):
        self.land_position = land_position
    
    def set_collision_info(self, collision_info):
        if collision_info.has_collided:
            if 'person' in collision_info.object_name or 'bird' in collision_info.object_name or 'dog' in collision_info.object_name:
                self.collision = 1

    def set_land_scenes(self, scenes):
        self.scenes = scenes
    def generate_result(self):
        try:
            self.land_duration = self.end_land_time - self.start_land_time
            self.land_duration = self.land_duration.to_sec()
        except:
            self.land_duration = 10


        tp_marker_position = [self.scenario.tp_marker.pose.x, self.scenario.tp_marker.pose.y]
        land_position = [self.land_position.x_val, self.land_position.y_val]
        self.land_deviation = distance_2d(tp_marker_position, land_position)
        if abs(-self.land_position.z_val - 1) < 1 and self.land_deviation < 1.5:
            self.land_success = 1 
        else:
            self.land_success = 0
        min_fp_distance = None
        for fp_marker in self.scenario.fp_markers:
            fp_marker_position = [fp_marker.pose.x, fp_marker.pose.y]
            dist = distance_2d(fp_marker_position, land_position)
            if not min_fp_distance or dist < min_fp_distance:
                min_fp_distance = dist
        
        self.land_fp_dist = min_fp_distance


    def get_result(self):
        return {'land_duration': self.land_duration,
                'land_deviation': self.land_deviation,
                'land_success': self.land_success,
                'land_fp_dist': self.land_fp_dist,
                'collision': self.collision,
                }


def start_scenario(scenario, env):
    env.set_scenario(scenario)
    env.load_scenario()
    time.sleep(3)
    env.set_segmentation()


def cal_fitness(scenario_result, scenario_rewards=None):
    pass

def get_state(env, scenario, npc_idx):
    marker_position = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y)
    actor_pose = env.get_pose(env.dynamic_objects[npc_idx])
    actor_position = (actor_pose.position.x_val, actor_pose.position.y_val)
    drone_current_pose = env.client.simGetVehiclePose()
    drone_current_position = [drone_current_pose.position.x_val, drone_current_pose.position.y_val]

    state  = (actor_position[0]-marker_position[0],actor_position[1]-marker_position[1],
                drone_current_position[0]-marker_position[0],drone_current_position[1]-marker_position[1])
    
    return state

class TestFramework:
    def __init__(self, pop_size, generations, mutation_rate, host_ip, scenario_topic, map_name, result_save_path='/home/yao/Documents/random_result'):
        self.pop_size = pop_size
        self.generations = generations
        self.mutation_rate = mutation_rate

        # create multiple airsim connections for different threads to avoid IOLOOP error
        self.env = AirSimEnv(host_ip=host_ip, map_name=map_name)
        self.env_cb = AirSimEnv(host_ip=host_ip, map_name=map_name)
        self.result_save_path = result_save_path
        if not os.path.exists(self.result_save_path):
            os.mkdir(self.result_save_path)

        self.scenario_sub = rospy.Subscriber(scenario_topic, Scenario, self.cb_scenario)
        self.scenario_pub = rospy.Publisher(scenario_topic, Scenario, queue_size=10)

        self.drone_target_sub = rospy.Subscriber('/waypoint_generator/waypoints', Path, self.cb_drone_target)
        self.drone_pose_sub = rospy.Subscriber('/camera_world_pose', PoseStamped, self.cb_drone_pose)
        self.scene_sub = rospy.Subscriber('/detection/debug', Image, self.cb_scene)
        self.cv_bridge = CvBridge()

        self.current_scenario_result = None
        self.current_drone_traj = []
        self.current_scenario_scenes = []
        self.last_scene_time = None
        self.generation_result = []
        self.scenario_status = 'idle'

    def cb_drone_pose(self, pose_msg):
        self.current_drone_traj.append([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
    
    def cb_scene(self, image_msg):
        current_time = rospy.Time.now().to_sec()
        if self.current_scenario_result is not None and (self.last_scene_time is None or current_time - self.last_scene_time >= 2):
            image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            self.current_scenario_scenes.append(image)
            self.last_scene_time = current_time
    
    def save_scenario_result(self, generation_id, scenario_id):
        print(self.current_scenario_result.get_result())
        # rospy.loginfo("current scenario result: {}".format(str(self.current_scenario_result.get_result())))
        result_dict = {'scenario': self.current_scenario_result.scenario.to_json(),
                       'land_result': self.current_scenario_result.get_result(),
                       'trajectory': self.current_drone_traj,
                       'scenes': self.current_scenario_scenes}
        with open(os.path.join(self.result_save_path, '{}-{}.pickle'.format(generation_id, scenario_id)), 'wb') as f:
            pickle.dump(result_dict, f)

    def reset_scenario_data(self):
        self.current_scenario_result = None
        self.current_drone_traj = []
        self.current_scenario_scenes = []

    def cb_drone_target(self, pose_msg):
        if self.current_scenario_result is not None and self.current_scenario_result.start_land_time is None:
            self.current_scenario_result.set_start_land_time(pose_msg.header.stamp)

    def cb_scenario(self, scenario_msg):
        if scenario_msg.status == 1:
            scenario = ScenarioObj()
            scenario.load_from_msg(scenario_msg)
            self.current_scenario_result = ScenarioResult(scenario)
            self.scenario_status = 'running'
            rospy.loginfo('get current scenario')
        elif scenario_msg.status==4:
            self.scenario_status = 'onlanding'
            # for obj in self.env.scenario_objects:
            #     self.env.reset_npc(obj)
            #     time.sleep(1)
        elif scenario_msg.status == 2:
            rospy.loginfo('scenario finished')

            # self.env.client.moveToPositionAsync(0, 0, -1, 5).join()
            # self.env.client.landAsync()
            # self.env.client.armDisarm(False)
            # time.sleep(3)
            self.current_scenario_result.set_end_land_time(scenario_msg.header.stamp)
            self.current_scenario_result.set_land_position(self.env_cb.client.simGetVehiclePose().position)
            self.current_scenario_result.set_land_scenes(self.current_scenario_scenes)
            collision_info = self.env_cb.client.simGetCollisionInfo()
            self.current_scenario_result.set_collision_info(collision_info)
            # self.env.set_drone_pose(Pose(0, 0, 1, 0))
            self.env_cb.client.reset()
            print('landed')

            self.current_scenario_result.generate_result()
            self.scenario_status = 'end'
    
    def initial_population(self):
        scenarios = [sample_scenario(self.env.map_config) for _ in range(self.pop_size)]
        return scenarios



    def run(self):
        pass

    def mutate(self):
        pass

    def crossover(self):
        pass




            







