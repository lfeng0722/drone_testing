#!/usr/bin/env python3
import rospy
from airsim_simulation.sim_env import AirSimEnv
from airsim_simulation.scenario_utils import sample_scenario, sample_test_scenario
from airsim_simulation.configuration import ACTOR_TYPE_DICT
from simulation.msg import Scenario, Marker, Actor, ScenarioPose, TimeOfDay, Weather
import time

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

if __name__ == "__main__":
    rospy.init_node("scenario_sampler")
    scenario_topic = rospy.get_param('~scenario_topic', '/simulation/scenario')

    scenario_pub = rospy.Publisher(scenario_topic, Scenario, queue_size=10)
    time.sleep(5)


    scenario = sample_test_scenario()
    print(scenario)
    env = AirSimEnv('cv', '10.6.37.180')
    env.set_scenario(scenario)
    env.load_scenario()

    scenario_msg = scenario_to_msg(scenario)
    scenario_pub.publish(scenario_msg)
    print('publish scenario')


    rospy.spin()