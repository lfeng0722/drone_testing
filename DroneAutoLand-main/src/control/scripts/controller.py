#!/usr/bin/env python3

from control.airsim_control import AirSimController
from simulation.msg import ScenarioPose, Scenario
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, Int32

import math
import rospy

class Controller:
    def __init__(self, drone_controller, local_target_topic, global_target_topic, scenario_topic):
        self.drone_controller = drone_controller
        self.local_target_sub = rospy.Subscriber(local_target_topic, PositionCommand, self.cb_local_target, queue_size=10)
        self.global_target_sub = rospy.Subscriber(global_target_topic, ScenarioPose, self.cb_global_target)
        self.scenario_sub = rospy.Subscriber(scenario_topic, Scenario, self.cb_scenario)
        self.scenario_pub = rospy.Subscriber(scenario_topic, Scenario, queue_size=10)

        self.takeoff_sub = rospy.Subscriber("/drone_action/takeoff", Int32, self.cb_takeoff)
        # land sub is not used in test generation exp. 
        # the landing process is controlled by test generation script instead.
        self.land_sub = rospy.Subscriber("/drone_action/land", Bool, self.cb_land)
        self.last_local_target = None
        self.enable = False
        self.current_scenario = None
    
    def cb_scenario(self, msg):
        if msg.status == 1:
            self.current_scenario = msg
            print('set scenario')
        elif msg.status == 2:
            print('disable control')
            self.enable = False

    #     rospy.loginfo("Scenario received. Takeoff.")
    #     self.drone_controller.takeoff()

    def cb_takeoff(self, msg):
        rospy.loginfo("Takeoff received. Takeoff.")
        self.drone_controller.takeoff(msg.data)
        self.enable = True


    def cb_land(self, msg):
        rospy.loginfo("Land received. Land.")
        self.drone_controller.land()
        # print('update scenario')
        # self.current_scenario.status = 2
        # self.current_scenario.header.stamp = rospy.Time.now()
        # self.scenario_pub.publish(self.current_scenario)
        # self.enable = False

    def cb_local_target(self, cmd_msg):
        # rospy.loginfo("Local target received. Move.")
        if self.enable:
            # print('move to local target')
            velocity = cmd_msg.velocity
            position = cmd_msg.position
            pose = ScenarioPose(position.y, position.x, -position.z, 0)
            if self.last_local_target is None or pose.x != self.last_local_target.x or \
                pose.y!= self.last_local_target.y or pose.z!= self:
                self.last_local_target = pose
            # if self.last_local_target is None or self.last_local_target.position.x != pose.x or \
            #     self.last_local_target.position.y != pose.y or self.last_local_target.position.z!= pose.z:
            # print('local target: ', pose)
            # self.last_local_target = pose
            # yaw = cmd_msg.yaw
            # scenario_pose = Pose(position.x, position.y, position.z, math.degrees(yaw))
                self.drone_controller.move_to_local(pose, velocity)
            # self.drone_controller.move_to_global(pose, 1)

    
    def cb_global_target(self, pose_msg):
        # rospy.loginfo(pose_msg)
        rospy.loginfo("Global target received. Move.")
        self.drone_controller.move_to_global(pose_msg, 1)


if __name__ == "__main__":
    rospy.init_node("controller")

    host_ip = rospy.get_param('~host_ip', '10.6.37.180')
    drone_name = rospy.get_param('~drone_name', 'Copter')

    local_target_topic = rospy.get_param('~local_target_topic', '/planning/pos_cmd')
    global_target_topic = rospy.get_param('~global_target_topic', '/decision/global_location')
    scenario_topic = rospy.get_param('~scenario_topic', '/simulation/scenario')

    airsim_control = AirSimController(host_ip, drone_name)
    controller = Controller(airsim_control, local_target_topic, global_target_topic, scenario_topic)


    rospy.spin()