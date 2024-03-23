from control.drone_control import DroneController
import mavros_msgs.msg
import mavros_msgs.srv
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
import geometry_msgs.msg
import time

MAX_TRY_TIMES = 20

class TopicService:
    def __init__(self, name: str, classType):
        self.__name = name
        self.__classType = classType
        self.__data = None

    def set_data(self, data):
        self.__data = data

    def get_data(self):
        return self.__data

    def get_type(self):
        return self.__classType

    def get_name(self):
        return self.__name

class ArdupilotController(DroneController):
    def __init__(self, drone_name='naga'):
        super(ArdupilotController, self).__init__(drone_name)
        self.armed = False
        self.guided = False
        self.mode = None
        self.system_status = 0
        self.drone_pose = None

        # self.TOPIC_STATE = TopicService("/{}/mavros/state".format(drone_name), mavros_msgs.msg.State)
        self.SERVICE_ARM = TopicService("/{}/mavros/cmd/arming".format(drone_name), mavros_msgs.srv.CommandBool)
        self.SERVICE_SET_MODE = TopicService("/{}/mavros/set_mode".format(drone_name), mavros_msgs.srv.SetMode)        
        self.SERVICE_TAKEOFF = TopicService('/{}/mavros/cmd/takeoff'.format(drone_name), mavros_msgs.srv.CommandTOL)
        self.SERVICE_SET_PARAM = TopicService("/naga/mavros/param/set", mavros_msgs.srv.ParamSet)
        self.SERVICE_GET_PARAM = TopicService("/naga/mavros/param/get", mavros_msgs.srv.ParamGet)
        self.pose_pub = rospy.Publisher('/{}/mavros/setpoint_position/local'.format(drone_name), PoseStamped, queue_size=1000)
        self.state_sub = rospy.Subscriber("/{}/mavros/state".format(drone_name), mavros_msgs.msg.State, self.state_cb)
        self.pose_sub = rospy.Subscriber("/{}/mavros/local_position/pose".format(drone_name), PoseStamped, self.pose_cb)
        # self.thread_param_updater = threading.Timer(0, self.update_parameters_from_topic)
        # self.thread_param_updater.daemon = True
        # self.thread_param_updater.start()

    def pose_cb(self, pose_msg):
        if pose_msg:
            self.drone_pose = pose_msg.pose

    def state_cb(self, state_msg):
        if state_msg:
            self.mode = state_msg.mode
            self.armed = state_msg.armed
            self.guided = state_msg.guided
            self.system_status = state_msg.system_status


    @staticmethod
    def topic_publisher(topic: TopicService):
        print(topic.get_name(), topic.get_type(), topic.get_data())
        pub = rospy.Publisher(topic.get_name(), topic.get_type(), queue_size=10)
        pub.publish(topic.get_data())
        # print(topic.get_data())

    @staticmethod
    def topic_subscriber(topic: TopicService):
        rospy.Subscriber(topic.getsuccess_name(), topic.get_type(), topic.set_data)

    @staticmethod
    def service_caller(service: TopicService, timeout=30):
        try:
            srv = service.get_name()
            typ = service.get_type()
            data = service.get_data()

            rospy.loginfo("waiting for ROS service:" + srv)
            rospy.wait_for_service(srv, timeout=timeout)
            rospy.loginfo("ROS service is up:" + srv)
            call_srv = rospy.ServiceProxy(srv, typ)
            return call_srv(data)
        except rospy.ROSException as e:
            print("ROS ERROR:", e)
        except rospy.ROSInternalException as e:
            print("ROS ERROR:", e)
        except KeyError as e:
            print("ERROR:", e)
        return None
    

    def takeoff(self, height=10, velocity=3, min_pitch=0, yaw=0, latitude=0, longitude=0):
        if self.mode == 'LAND' or self.mode == 'STABILIZE' or self.mode == None:
            self.change_mode('GUIDED')
            time.sleep(3)
            print(self.mode)

        arm_status = False
        print('drone arming')
        try_times = 0

        while not arm_status:
            arm_status, _ = self.arm(True)
            if not arm_status:
                time.sleep(3)
                try_times += 1
            else:
                break
            
            if try_times == MAX_TRY_TIMES:
                return False
        
        data = mavros_msgs.srv.CommandTOLRequest()
        data.min_pitch = min_pitch
        data.yaw = yaw
        data.latitude = latitude
        data.longitude = longitude
        data.altitude = height
        try_times = 0
        success = False
        while True:
            print('taking off')
            self.SERVICE_TAKEOFF.set_data(data)
            result = self.service_caller(self.SERVICE_TAKEOFF, timeout=30)
            if result:
                success = True
                break
            else:
                if try_times > MAX_TRY_TIMES:
                    break

                arm_status, _ = self.arm(True)
                try_times += 1
                time.sleep(1)
        
        if not success:
            return False
        
        success = False
        while True:
            if abs(self.drone_pose.position.z - height) < 0.3:
                success = True
                print("Finish taking off")
                break    

        return success
    
    def set_param(self, param: str, value_integer: int, value_real: float):
        data = mavros_msgs.srv.ParamSetRequest()
        data.param_id = param
        data.value.integer = value_integer
        data.value.real = value_real
        self.SERVICE_SET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_SET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real
    
    def get_param(self, param: str):
        data = mavros_msgs.srv.ParamGetRequest()
        data.param_id = param
        self.SERVICE_GET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_GET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    def arm(self, status: bool):
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        self.SERVICE_ARM.set_data(data)
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        return result.success, result.result

    def change_mode(self, mode: str):
        print('set mode: ', mode)
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        self.SERVICE_SET_MODE.set_data(data)
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent

    def move_to_local(self, position, angle=0):
        # print("Moving to: ", position)
        data = PoseStamped()
        position = geometry_msgs.msg.Point(x=position[0],y=position[1],z=position[2])
        q1 = Quaternion(axis=[0., 0., 1.], angle=angle)

        orientation = geometry_msgs.msg.Quaternion(x=q1.elements[1],y=q1.elements[2],z=q1.elements[3],w=q1.elements[0])
        pose = geometry_msgs.msg.Pose(position=position, orientation=orientation)
        data.pose = pose
        # print(data)
        self.pose_pub.publish(data)
        # time.sleep(1)
    

    def land(self):
        if self.mode != 'LAND':
            self.change_mode('LAND')

        while True:
            if self.system_status == 3:
                print("Finish landing")
                break
            time.sleep(1)
