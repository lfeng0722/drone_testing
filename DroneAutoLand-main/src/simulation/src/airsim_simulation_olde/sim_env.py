# import rospy
import airsim
import random
import numpy as np
from pyquaternion import Quaternion
import math
import time
import cv2
try:
    from airsim_simulation.configuration import OBJ_DATABASE, ACTOR_TYPE_DICT
    from airsim_simulation.components import Pose
    from airsim_simulation.scenario_utils import distance_2d
except:
    from configuration import OBJ_DATABASE, ACTOR_TYPE_DICT
    from components import Pose
    from scenario_utils import distance_2d

"""
    AirSim environment APIs
"""
class AirSimEnv():

    def __init__(self, sim_mode='drone', host_ip='localhost') -> None:
        # define airsim API client
        self.sim_mode = sim_mode
        if sim_mode == 'cv':
            self.client = airsim.VehicleClient(host_ip)
            # set downward camera
            self.client.simSetCameraPose('0', airsim.Pose(airsim.Vector3r(0, 0, 0),
                                         airsim.to_quaternion(math.radians(-90), 0, 0)))
        else:
            self.client = airsim.MultirotorClient(host_ip)
            self.client2 = airsim.MultirotorClient(host_ip)
            self.client2.confirmConnection()
            self.client2.enableApiControl(True)
            self.client2.armDisarm(True)
        self.client.simEnableWeather(True)
        self.current_scenario = None
        self.dynamic_objects = []
        self.dynamic_objects_pool = {'person_1': ['person_1-0', 'person_1-1', 'person_1-2'],
                                     'person_2': ['person_2-0', 'person_2-1', 'person_2-2'],
                                     # 'deer_1': ['deer_1-0', 'deer_1-1', 'deer_1-2'],
                                     'bird_1': ['bird_1-0', 'bird_1-1', 'bird_1-2'],
                                     'dog_1': ['dog_1-0', 'dog-1-1', 'dog_1-2']}
                                     # 'zebra_1': ['zebra_1-0', 'zebra_1-1', 'zebra_1-2']}


    def surrograte_trianing(self, maker_position, UAV_spawn_position):
        #
        orientation = airsim.to_quaternion(math.radians(0), math.radians(0), math.radians(0))
        position = airsim.Vector3r(UAV_spawn_position[0], UAV_spawn_position[1], -3)
        pose = airsim.Pose(position_val=position, orientation_val=orientation)
        self.client2.simSetVehiclePose(pose, ignore_collision= True )
        time.sleep(1)
        self.client2.takeoffAsync(20).join()
        time.sleep(1)
        self.client2.moveToPositionAsync(position.x_val, position.y_val, -15, 5).join()
        self.client2.moveToPositionAsync(maker_position.pose.x, maker_position.pose.y, -0, 5)
    def set_marker(self, scenario_pose, marker_name='cube_marker0'):
        orientation = airsim.to_quaternion(math.radians(0), math.radians(0), math.radians(scenario_pose.angle))
        # orientation = airsim.Quaternionr(w_val=1, x_val=0, y_val=0, z_val=0)

        position = airsim.Vector3r(scenario_pose.x, scenario_pose.y, -scenario_pose.z)
        pose = airsim.Pose(position_val=position, orientation_val=orientation)

        success = self.client.simSetObjectPose(marker_name, pose)

        marker_pose = self.client.simGetObjectPose(marker_name)
        # print(success, marker_pose)
        return marker_pose
    
    def get_collision_info(self):
        return self.client.simGetCollisionInfo().object_name

    def get_obj_name(self):
        return self.client.simListSceneObjects()
    def set_weather(self, params):
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Rain, min(1, params[0]))
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Roadwetness, min(1, params[1]))
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Snow, min(1, params[2]))
        self.client.simSetWeatherParameter(airsim.WeatherParameter.RoadSnow, min(1, params[3]))
        self.client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, min(1, params[4]))
        self.client.simSetWeatherParameter(airsim.WeatherParameter.RoadLeaf, min(1, params[5]))
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Dust, min(1, params[6]))
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Fog, min(1, params[7])) 
        if self.sim_mode != 'cv':    
            wind = airsim.Vector3r(min(5, params[8]*5), min(5, params[8]*5), 0)
            self.client.simSetWind(wind)

        # set light

    def reset_env(self):
        # dynamic_objects = ['BP_SUV_2', 'npc_vehicle_2', 'npc_person_1', 'DeerBP_2', 'npc_bird_1', 'npc_person_2', 'npc_person_3']
        for i, obj in enumerate(self.dynamic_objects):
            # success = self.client.simDestroyObject(obj)
            success=False
            while not success:
                success = self.client.simSetObjectPose(obj, airsim.Pose(airsim.Vector3r(100, 100 + i * 5, 0.5),
                 airsim.Quaternionr(w_val=1, x_val=0, y_val=0, z_val=0)))
            print('destroy actor', obj, success)
            if success:
                npc_type = obj.split('-')[0]
                self.dynamic_objects_pool[npc_type].append(obj)
        self.dynamic_objects = []
        # reset markers
        # markers = []
        # markers.append('plane_marker{}'.format())
        markers = ['plane_marker{}'.format(i) for i in range(3)]
        # markers.append('plane_marker0')

        for i, marker in enumerate(markers):
            # self.client.simDestroyObject(marker)
            self.client.simSetObjectPose(marker, airsim.Pose(airsim.Vector3r(100, 100 + 5 * i, 1),
             airsim.Quaternionr(w_val=1, x_val=0, y_val=0, z_val=0)))
            # print(marker, success)
        print("set env finished", self.dynamic_objects_pool)
        # reset weather
    
    def get_current_scene(self, camera_name='0', image_type=0, image_encoding='rgb'):
        if image_type == 0:
            responses = self.client.simGetImages([airsim.ImageRequest(camera_name, image_type, False, False)])
        elif image_type == 5:
            responses = self.client.simGetImages([airsim.ImageRequest(camera_name, image_type, False, False)])

        response = responses[0]
        # get numpy array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

        # reshape array to 4 channel image array H X W X 4
        img = img1d.reshape(response.height, response.width, 3)
        if image_encoding == 'bgr' and image_type == 0:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            # img = img[:, :, ::-1] 
        camera_body_pose = airsim.Pose(response.camera_position, response.camera_orientation)

        
        return img, camera_body_pose
    
    def get_segmentation_mapping(self):
        colors = {}
        requests = airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)

        for cls_id in range(20):
            # map every asset to cls_id and extract the single RGB value produced
            self.client.simSetSegmentationObjectID(".*", cls_id, is_name_regex=True)
            response = self.client.simGetImages([requests])[0]
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(response.height, response.width, 3)

            color = tuple(np.unique(img_rgb.reshape(-1, img_rgb.shape[-1]), axis=0)[0])
            print(f"{cls_id}\t{color}")
            colors[cls_id] = color
        
        return colors

    def set_segmentation(self):
        self.client.simSetSegmentationObjectID("[\w]*", -1, True)

        filtered_objs = ['House', 'Fir', 'Fence', 'Car', 'Power_line', 'Roof', 'Swimming', 'Rock', 'Hedge', 'Wall', 'Tree', 'npc', 'SUV', 'Birch']
        seg_id = 1
        for obj in filtered_objs:
            self.client.simSetSegmentationObjectID("[\w]*{}[\w]*".format(obj), seg_id, True)
            seg_id += 1

        for i in range(4):
            self.client.simSetSegmentationObjectID("plane_marker{}".format(i), seg_id)
        
        # for actor in self.dynamic_objects:
        #     self.client.simSetSegmentationObjectID(actor, 2)
        # self.client.simSetSegmentationObjectID("npc_person_2", 11)
        # self.client.simSetSegmentationObjectID("npc_person_3", 11)
        return seg_id

    def add_npc(self, npc_type, scenario_pose):
        # if npc_type not in OBJ_DATABASE.keys():
        #     return False, None
        # bp_name = OBJ_DATABASE[npc_type]
        # npc_name = 'npc_{}_{}'.format(npc_type, random.randint(0, 10000))
        try:
            npc_name = self.dynamic_objects_pool[npc_type].pop(0)
        except:
            return False, None
        print('add actor: ', npc_name, scenario_pose.x, scenario_pose.y, scenario_pose.z)

        # angle = 0
        # q1 = Quaternion(axis=[0., 0., 1.], angle=scenario_pose.angle)
        # orientation = airsim.Quaternionr(w_val=q1.elements[0], x_val=q1.elements[1], y_val=q1.elements[2], z_val=q1.elements[3])
        # pose = airsim.Pose(airsim.Vector3r(scenario_pose.x, scenario_pose.y, -scenario_pose.z), orientation)
        pose = airsim.Pose(airsim.Vector3r(scenario_pose.x, scenario_pose.y, -scenario_pose.z))

        # success = self.client.simSetObjectPose(npc_name, pose)
        # print(npc_name, bp_name)
        # obj_name = None

        # obj_name = self.client.simSpawnObject(object_name=npc_name, asset_name=bp_name, pose=pose, scale=airsim.Vector3r(1, 1, 1),
        #                                     is_blueprint=True)
        success = self.client.simSetObjectPose(npc_name, pose)

        # pose_new = airsim.Pose(airsim.Vector3r(scenario_pose.x, scenario_pose.y, -scenario_pose.z), orientation)
        # self.client.simSetObjectPose(obj_name, pose_new)
        # print(obj_name)
        if success:
            self.dynamic_objects.append(npc_name)
            return True, npc_name
        else:
            return False, None
    
    def set_drone_pose(self, scenario_pose):
        if self.sim_mode == 'cv':
            drone_name = ''
        else:
            drone_name = 'Copter'


        self.client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(scenario_pose.x, scenario_pose.y, -scenario_pose.z),
            airsim.to_quaternion(0, 0, math.radians(scenario_pose.angle))), True, vehicle_name=drone_name)
        if self.sim_mode != 'cv':
            self.client.armDisarm(False)
        return self.client.simGetVehiclePose(drone_name)
        

    def set_npc_pose(self, npc_name, scenario_pose):
        q1 = Quaternion(axis=[0., 0., 1.], angle=scenario_pose.angle)
        orientation = airsim.Quaternionr(w_val=q1.elements[0], x_val=q1.elements[1], y_val=q1.elements[2], z_val=q1.elements[3])

        pose = airsim.Pose(airsim.Vector3r(scenario_pose.x, scenario_pose.y, -scenario_pose.z), orientation)
        self.client.simSetObjectPose(npc_name, pose)


    def get_pose(self, obj_name='npc_person_1'):
        # print(npc_name)
        pose = self.client.simGetObjectPose(obj_name)
        # print(pose)
        return pose
    
    def move_npc_to(self, npc_name, scenario_pose, speed=0.5):
        try:
            angle = scenario_pose.angle
        except:
            angle = scenario_pose.yaw
        # q1 = Quaternion(axis=[0., 0., 1.], angle=angle)
        # orientation = airsim.Quaternionr(w_val=q1.elements[0], x_val=q1.elements[1], y_val=q1.elements[2], z_val=q1.elements[3])
        # pose = airsim.Pose(airsim.Vector3r(scenario_pose.x, scenario_pose.y, -scenario_pose.z), orientation)  
        curernt_pose = self.get_pose(npc_name)
        pose = airsim.Pose(airsim.Vector3r(round(scenario_pose.x, 2), round(scenario_pose.y, 2), curernt_pose.position.z_val))    
        #   
        # print(npc_name, speed)
        if 'vehicle' not in npc_name:
            npc_speed = speed * 300
            print("set npc speed: ", npc_speed)
            self.client.simSetNPCSpeed(npc_name, npc_speed)     
        result = self.client.simSetNPCMoveTo(npc_name, pose)
        print('{} move to ({} {} {})'.format(npc_name, round(scenario_pose.x, 2),
         round(scenario_pose.y, 2), round(-scenario_pose.z, 2)))

    def set_time_of_day(self, params):
        hour, minute = params[0], params[1]
        hour_time = int(hour * 24)
        min_time = int(minute * 60)
        sim_time = '2023-09-06 {}:{}:00'.format(hour_time, min_time)

        self.client.simSetTimeOfDay(True, start_datetime = sim_time, is_start_datetime_dst = True,
         celestial_clock_speed = 1, update_interval_secs = 1, move_sun = True)
    
    """
    The following movemnent functions don't consider the orientation of the npc. They only consider the change of X and Y values.
    """
    def move_npc_forward(self, npc_name, speed=0.5):
        npc_position = self.get_pose(npc_name)
        new_x = npc_position.position.x_val + 10
        scenario_pose = Pose(new_x, npc_position.position.y_val, -npc_position.position.z_val)
        self.move_npc_to(npc_name, scenario_pose, speed)
        # self.client.simSetObjectPose(npc_name, airsim.Pose(airsim.Vector3r(new_x, npc_position.position.y_val, npc_position.position.z_val)))

    def move_npc_backward(self, npc_name, speed=0.5):
        npc_position = self.get_pose(npc_name)
        new_x = npc_position.position.x_val - 10
        scenario_pose = Pose(new_x, npc_position.position.y_val, -npc_position.position.z_val)
        self.move_npc_to(npc_name, scenario_pose, speed)
        # self.client.simSetObjectPose(npc_name, airsim.Pose(airsim.Vector3r(new_x, npc_position.position.y_val, npc_position.position.z_val)))


    def move_npc_right(self, npc_name, speed):
        npc_position = self.get_pose(npc_name)
        new_y = npc_position.position.y_val - 10
        scenario_pose = Pose(npc_position.position.x_val, new_y, -npc_position.position.z_val)
        self.move_npc_to(npc_name, scenario_pose, speed)
        # self.client.simSetObjectPose(npc_name, airsim.Pose(airsim.Vector3r(npc_position.position.x_val, new_y, npc_position.position.z_val)))

    def move_npc_left(self, npc_name, speed):
        npc_position = self.get_pose(npc_name)
        new_y = npc_position.position.y_val + 10
        scenario_pose = Pose(npc_position.position.x_val, new_y, -npc_position.position.z_val)
        self.move_npc_to(npc_name, scenario_pose, speed)
        # self.client.simSetObjectPose(npc_name, airsim.Pose(airsim.Vector3r(npc_position.position.x_val, new_y, npc_position.position.z_val)))

    def get_uav_pose(self):
        return self.client.simGetVehiclePose()

    def set_npc_forward(self, npc_name, speed=0.5):
        npc_position = self.get_pose(npc_name)
        new_x = npc_position.position.x_val + 1
        scenario_pose = Pose(new_x, npc_position.position.y_val, -npc_position.position.z_val)
        # self.move_npc_to(npc_name, scenario_pose, speed)
        self.client.simSetObjectPose(npc_name, airsim.Pose(airsim.Vector3r(new_x, npc_position.position.y_val, npc_position.position.z_val)))

    def set_npc_backward(self, npc_name, speed=0.5):
        npc_position = self.get_pose(npc_name)
        new_x = npc_position.position.x_val - 1
        scenario_pose = Pose(new_x, npc_position.position.y_val, -npc_position.position.z_val)
        # self.move_npc_to(npc_name, scenario_pose, speed)
        self.client.simSetObjectPose(npc_name, airsim.Pose(airsim.Vector3r(new_x, npc_position.position.y_val, npc_position.position.z_val)))


    def set_npc_right(self, npc_name, speed):
        npc_position = self.get_pose(npc_name)
        new_y = npc_position.position.y_val - 1
        scenario_pose = Pose(npc_position.position.x_val, new_y, -npc_position.position.z_val)
        # self.move_npc_to(npc_name, scenario_pose, speed)
        self.client.simSetObjectPose(npc_name, airsim.Pose(airsim.Vector3r(npc_position.position.x_val, new_y, npc_position.position.z_val)))

    def set_npc_left(self, npc_name, speed):
        npc_position = self.get_pose(npc_name)
        new_y = npc_position.position.y_val + 1
        scenario_pose = Pose(npc_position.position.x_val, new_y, -npc_position.position.z_val)
        # self.move_npc_to(npc_name, scenario_pose, speed)
        self.client.simSetObjectPose(npc_name, airsim.Pose(
            airsim.Vector3r(npc_position.position.x_val, new_y, npc_position.position.z_val)))

    def validate_scenario(self, scenario):
        # validate and rectify the scenario in the simulation environment
        # we need to validate whether the markers can be placed on the ground,
        
        # validate_tp_marker
        while True:
            self.set_marker(scenario.tp_marker.pose, marker_name='plane_marker{}'.format(scenario.tp_marker.id))
            time.sleep(3)
            marker_pose = self.get_pose('plane_marker{}'.format(scenario.tp_marker.id))
            if abs(marker_pose.position.z_val) > 1:
                scenario.tp_marker.mutate() 
            else:
                break
        all_marker_pos = []
        pose = self.get_pose('plane_marker{}'.format(scenario.tp_marker.id))
        all_marker_pos.append((pose.position.x_val, pose.position.y_val))
        # set fp_markers
        while len(all_marker_pos) < len(scenario.fp_markers) + 1:
            while True:
                fp_marker = scenario.fp_markers[len(all_marker_pos)]
                self.set_marker(fp_marker.pose, marker_name='plane_marker{}'.format(fp_marker.id))
                time.sleep(3)
                marker_pose = self.get_pose('plane_marker{}'.format(fp_marker.id))
                pos = (marker_pose.position.x_val, marker_pose.position.y_val)
                if abs(marker_pose.position.z_val) > 1:
                    fp_marker.mutate() 
                elif all(distance_2d(pos, marker_pos) > 2 for marker_pos in all_marker_pos):
                    pose = self.get_pose('plane_marker{}'.format(fp_marker.id))
                    all_marker_pos.append(pos)
                    break
        
        return scenario

    def set_scenario(self, scenario):
        self.current_scenario = scenario
        print('current scenario: ', scenario)


    # set all actors at the initial position
    def load_scenario(self):
        scenario = self.current_scenario

        # set drone pose for cv and simple flight mode
        # if self.sim_mode != 'ardupilot':
        #     if self.sim_mode == 'cv':
        #         self.set_drone_pose(scenario.drone_start_pose)
        #     else:
        #         self.set_drone_pose(scenario.drone_start_pose)
        # create the landing scenario from the scenario configuration
        self.reset_env()
        time.sleep(1)
        self.set_marker(scenario.tp_marker.pose, marker_name='Cube2')
        # time.sleep(3)
        # print(self.get_pose('plane_marker{}'.format(scenario.tp_marker.id)))
        for marker in scenario.fp_markers:
            self.set_marker(marker.pose, marker_name='cube_marker{}'.format(marker.id))
        

        
        # set weather and time
        self.set_weather(scenario.weather.to_vec())
        self.set_time_of_day(scenario.time.to_vec())

        # set dynamic actors

        for actor in scenario.actors:
            print('{} init ({} {} {})'.format(actor.type, actor.start_pose.x, actor.start_pose.y, -actor.start_pose.z))
            if abs(actor.start_pose.x - 1) < 1 and abs(actor.start_pose.y - 1) < 1:
                actor.start_pose.x += random.uniform(3, 5)
                actor.start_pose.y += random.uniform(3, 5)
            success, _ = self.add_npc(ACTOR_TYPE_DICT[actor.type], actor.start_pose)
            if not success:
                actor.type = -1

        
    # if the actors are not static, move them to the destination
    def run_scenario(self):
        scenario = self.current_scenario
        # print('11111111111111111111',self.dynamic_objects)
        if len(scenario.actors) != 0:
            for i, actor in enumerate(scenario.actors):
                if actor.type == -1:
                    continue
                # print(actor.end_pose)
                print('actor', i)
                try:
                    self.move_npc_to(self.dynamic_objects[i], actor.end_pose, actor.speed)
                except:
                    pass


if __name__ == "__main__":
    env = AirSimEnv('cv', '10.6.37.180')
    env.add_npc('npc_person_1', Pose(0, 10, 0))
    # env.reset_env()
    # time.sleep(3)
    # status, npc_name = env.add_npc('person_2', 2, 2, 1)
    # time.sleep(5)
    # env.get_pose('npc_vehicle_2')
    # env.client.simSetNPCVehicleThrottle('npc_vehicle_2', throttle=-0.5)
    # env.client.simSetNPCVehicleSteering('npc_vehicle_2', steering=0.5)
    # time.sleep(1)
    # env.client.simSetNPCVehicleThrottle('npc_vehicle_2', throttle=0)

    # env.npc_move_to('npc_vehicle_2', -1.89, 30, speed=300)
    # time.sleep(1)
    # env.set_marker(4, 2)
    # status, npc_name = env.add_npc('person_2', 0, 0, 1)
    # time.sleep(5)
    # env.get_npc_pose(npc_name)

    # if status:
    #     env.npc_move_to(npc_name, 10, 10, speed=300)
    # env.get_npc_pose('npc_deer_0')
    # env.set_npc('npc_person_1', 0, 0, -1)
    # env.set_time_of_day([0.2, 0.8])