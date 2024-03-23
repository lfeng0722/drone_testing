#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from marker_detection.msg import MarkerDetection
from simulation.msg import Scenario
from landing_decision.global_decision import GlobalDecisionMaker
from simulation.msg import ScenarioPose
from std_msgs.msg import Bool, Int32
import math
import time
import mavros_msgs.msg
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import CommandCode

def restart_ardu():
    rospy.wait_for_service('/naga/mavros/cmd/command')
    try:
        send_command = rospy.ServiceProxy('/naga/mavros/cmd/command', CommandLong)
        response = send_command(
            broadcast=False,
            command=CommandCode.PREFLIGHT_REBOOT_SHUTDOWN,
            confirmation=0,
            param1=1,  # Reboot autopilot
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        rospy.loginfo("Ardupilot restarted!!!!!")
        return response.success

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
def create_path(landing_location):
    # Initialize path message
    path = Path()

    # Set the frame ID (this is often 'map' or another reference frame)
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()

    # Create a sequence of poses

    pose = PoseStamped()

    pose.header.frame_id = "world"

    pose.pose.position.x = landing_location[0]
    pose.pose.position.y = landing_location[1]
    pose.pose.position.z = landing_location[2]  # For a 2D path on the ground, z is usually 0

    pose.pose.orientation.w = 1

    # If you also want to specify orientation, you can set it here.
    # For simplicity, we are keeping the default orientation (0, 0, 0, 1 in quaternion)
    print('set landing target: ', landing_location)
    path.poses.append(pose)

    return path

def distance_2d(point1, point2):
    """Calculate the Euclidean distance between two points in 2D space."""
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

class DecisionMaker:
    def __init__(self, search_strategy, post_processing_strategy,
                  scenario_topic, detection_topic, landing_location_topic, global_location_topic, drone_pose_topic):

        self.search_strategy = search_strategy
        self.post_processing_strategy = post_processing_strategy
        self.scenario_sub = rospy.Subscriber(scenario_topic, Scenario, self.cb_scenario)
        self.scenario_pub = rospy.Publisher(scenario_topic, Scenario, queue_size=10)
        self.detection_sub = rospy.Subscriber(detection_topic, MarkerDetection, self.cb_detection)
        self.drone_pose_sub = rospy.Subscriber(drone_pose_topic, PoseStamped, self.cb_drone_pose)

        self.landing_location_pub = rospy.Publisher(landing_location_topic, Path, queue_size=10)
        self.global_location_pub = rospy.Publisher(global_location_topic, ScenarioPose, queue_size=10)
        self.takeoff_pub = rospy.Publisher("/drone_action/takeoff", Int32, queue_size=10)
        self.land_pub = rospy.Publisher("/drone_action/land", Bool, queue_size=10)
        self.land_sub = rospy.Subscriber("/naga/mavros/state", mavros_msgs.msg.State, self.cb_state)
        self.planner_pub = rospy.Publisher("/planning/enable", Bool, queue_size=10)
        # self.final_landing_pub =rospy.Publisher('/drone_action/landingpoint',Int32MultiArray,queue_size = 10)
        self.fcu_sub = rospy.Subscriber("/naga/mavros/statustext/recv", mavros_msgs.msg.StatusText, self.cb_fcu)
        self.drone_position = None
        self.drone_position_updata_time = None
        self.current_scenario = None
        self.search_height = 10
        self.landing_height = 1

        self.global_decision_marker =  GlobalDecisionMaker(self.search_height)
        self.global_trajectory = None
        self.drone_status = 'static'
        self.drone_system_status = 1
        
        self.detected_markers = []
        self.current_landing_location = None

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while rospy.is_shutdown() is False:
            # rospy.loginfo("current drone status: {}".format(self.drone_status))
            if (self.current_scenario is not None) and (not self.global_trajectory) and (self.drone_status == 'static'):
                # wait the update of drone system status
                time.sleep(0.5)
                # if self.drone_system_status == 1 or self.drone_system_status == 2:
                # print(self.current_scenario is None, self.drone_status )
                self.global_trajectory = self.global_decision_marker.generate_global_trajectory(self.current_scenario)
                # print(self.current_scenario is None, self.drone_status)
                self.takeoff_pub.publish(Int32(self.search_height))
                rospy.loginfo("start takeoff")
                self.drone_status = 'in takeoff'
            
            if self.drone_status == 'in takeoff':
                # if self.drone_position:
                #     print('current drone height: {}'.format(self.drone_position.z))
                # print('1111111111111',-self.drone_position.z)
                # print('2222222222222', self.search_height)
                if self.drone_position and abs(-self.drone_position.z - self.search_height) < 2:
                    self.drone_status = 'finish takeoff'
                    time.sleep(5)
            
            if self.drone_status == 'finish takeoff':
                rospy.loginfo("finish takeoff. Start searching")
                # while True:
                #     if self.drone_system_status == 2 or self.drone_system_status == 1:
                #         break
                #     time.sleep(0.5)
                # self.global_location_pub.publish(self.global_trajectory[0])
                self.planner_pub.publish(Bool(True))
                time.sleep(0.5)
                print("moving to global location", self.global_trajectory[0].x, self.global_trajectory[0].y, self.global_trajectory[0].z)
                target_path = create_path([self.global_trajectory[0].y, self.global_trajectory[0].x, -self.global_trajectory[0].z])
                print("set target to global GPS location")
                self.landing_location_pub.publish(target_path)
                self.drone_status = 'in searching'
            
            # update next global trajectory and post-process on marker detection result to get the landing location
            if self.drone_status == 'in searching':
                if len(self.detected_markers) != 0:
                    detected_marker = self.detected_markers.pop(0)
                    if self.current_landing_location is None and detected_marker.confidence >= 0.8 and detected_marker.id == 0:
                        # convert ned coordinates to enu coordinates
                        landing_location = [detected_marker.pose.position.y, detected_marker.pose.position.x, self.landing_height]
                        # rospy.loginfo("set landing target: " + str([landing_location[1], landing_location[0], 1]))
                        self.current_landing_location = landing_location[:2]
                        print("found landing target")
                        self.landing_location_pub.publish(create_path(landing_location))
                    # elif self.current_landing_location is not None and detected_marker.confidence < 0.8 and -self.drone_position.z > 5:
                    #     rospy.loginfo("marker lost")
                    #     landing_location = [self.drone_position.y, self.drone_position.x, 1]
                    #     # rospy.loginfo("set landing target: " + str([landing_location[1], landing_location[0], 1]))
                    #     self.current_landing_location = landing_location[:2]
                    #     self.landing_location_pub.publish(create_path(landing_location))
                    elif self.current_landing_location is not None and detected_marker.confidence >= 0.8:
                        location = [detected_marker.pose.position.y, detected_marker.pose.position.x, self.landing_height]
                        if distance_2d(self.current_landing_location, location[:2]) > 0.7:
                            self.current_landing_location = location[:2]
                            rospy.loginfo("update landing target: " + str([location[1], location[0], self.landing_height]))
                            # self.final_landing_pub.publish(location[:2])
                            self.landing_location_pub.publish(create_path(location))

                finish_land = False
                reset_NPC = False
                # if self.current_landing_location:
                #     print('11111111111111111111111111', distance_2d(self.current_landing_location,[self.drone_position.y, self.drone_position.x]))
                #     print('222222222222222222222222222', abs(-self.drone_position.z - self.landing_height))
                if self.current_landing_location and distance_2d(self.current_landing_location,
                    [self.drone_position.y, self.drone_position.x]) < 0.5 and abs(-self.drone_position.z - self.landing_height) < 1:
                    rospy.loginfo("close to the landing target. Finish landing")
                    finish_land = True
                    reset_NPC = True
                else:
                    current_time = rospy.Time.now()
                    duration = current_time - self.drone_position_updata_time
                    # rospy.loginfo("pose update duration: " + str(duration.to_sec()))
                    if duration.to_sec() >15 and duration.to_sec() < 20:
                        reset_NPC = True

                    if duration.to_sec() > 20:
                        rospy.loginfo("No movement command for long time. Finish landing")
                        finish_land = True

                if reset_NPC:
                    self.current_scenario.status = 4
                    self.scenario_pub.publish(self.current_scenario)
                if finish_land:
                    # disable ego-planner

                    self.land_pub.publish(Bool(True))
                    self.planner_pub.publish(Bool(False))
                    self.drone_status = 'final landing phase'

            if self.drone_status == 'finish landing':

                # add a new state to check whether the drone is landed on the ground

                self.current_scenario.status = 2
                self.current_scenario.header.stamp = rospy.Time.now()
                self.scenario_pub.publish(self.current_scenario)

                self.drone_status = 'static'
                self.current_scenario = None
                self.global_trajectory = None
                self.detected_markers = []
                self.current_landing_location = None
                finish_land = False
                rospy.loginfo("finish landing")
                restart_ardu()
                # time.sleep(5)


        
            rate.sleep()
    
    def cb_detection(self, detection_msg):
        if self.drone_status == 'in searching' and detection_msg.confidence != -1:
            self.detected_markers.append(detection_msg)
        else:
            self.detected_markers = []

    def cb_drone_pose(self, drone_pose_msg):
        if self.drone_position is None:
            self.drone_position = drone_pose_msg.pose.position
            self.drone_position_updata_time = drone_pose_msg.header.stamp
        elif abs(drone_pose_msg.pose.position.x - self.drone_position.x) > 0.15 or abs(drone_pose_msg.pose.position.y - self.drone_position.y) > 0.15 \
            or abs(drone_pose_msg.pose.position.z - self.drone_position.z) > 0.15:
            # rospy.loginfo("drone position changed", )
            self.drone_position = drone_pose_msg.pose.position
            self.drone_position_updata_time = drone_pose_msg.header.stamp
    
    def cb_scenario(self, scenario):
        # generate global search trajectory based on GPS pose of the scenario
        if scenario.status == 1:
            print('get scenario')
            self.current_scenario = scenario

    def cb_state(self, state_msg):
        if state_msg:
            # self.drone_system_status = state_msg.system_status
            if self.drone_status == 'final landing phase' and state_msg.system_status == 3:
                print("Landed.")
                self.drone_status = 'finish landing'
            # elif self.drone_status == 'static':
            #     print('current system status: ', self.drone_system_status)

    def cb_fcu(self, fcu_msg):
        # print('333333333333333333333',fcu_msg)
        if  'GPS Glitch' in fcu_msg.text and 'cleared' not in fcu_msg.text:
            self.drone_system_status = 0
        elif  'Glitch cleared' in fcu_msg.text:
            # print(111)
            self.drone_system_status = 1
        elif 'yaw alignment complete' in fcu_msg.text:
            # print(222)
            self.drone_system_status = 2


def main():
    rospy.init_node('decision_maker')

    search_strategy = rospy.get_param('~search_strategy', 'straight')
    post_processing_strategy = rospy.get_param('~post_processing', 'none')

    scenario_topic = rospy.get_param('~scenario_topic', '/simulation/scenario')
    detection_topic = rospy.get_param('~detection_topic', '/detection/result')
    landing_location_topic = rospy.get_param('~target_location_topic', '/waypoint_generator/waypoints')
    global_location_topic = rospy.get_param('~global_location_topic', '/decision/global_location')
    drone_pose_topic = rospy.get_param('~drone_pose_topic', '/camera_world_pose')
    decision_marker = DecisionMaker(search_strategy, post_processing_strategy,
                                    scenario_topic, detection_topic, landing_location_topic,
                                    global_location_topic, drone_pose_topic)
    decision_marker.run()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down autoland processor...")
    except rospy.ROSException as e:
        if rospy.is_shutdown():
            pass
        else:
            rospy.logerr(e)

if __name__ == '__main__':
    main()