import airsim
import numpy as np
import os
import random
import time
from pyquaternion import Quaternion
import cv2
import random
from airsim.types import Pose, Vector3r, Quaternionr

client = airsim.MultirotorClient()
# client = airsim.MultirotorClient(ip='192.168.117.129')
client.confirmConnection()
client.simEnableWeather(True)
client.enableApiControl(True)
# client.simSetSegmentationObjectID('marker_mesh_9', 192)
# print(client.simGetMeshPositionVertexBuffers())


def add_car(x, y, z):
    print(client.simAddVehicle('vehicle',vehicle_type='PhysXCar', pose=airsim.Pose(position_val=airsim.Vector3r(x, y, z),
     orientation_val=airsim.Quaternionr(0, 0, 0, 1))))


def fly_to(x, y, z):
    client.moveToPositionAsync(x, y, z, 5)

def hover(drone_name):
    client.hoverAsync(drone_name)

def set_landing_scenario(marker_name="Maker_single", takeoff_height=10, weather_params=[0]*11):
    # reset_markers()
    drone_pose = respawn_drone()
    marker_pose = respawn_marker(marker_name=marker_name)

    if weather_params is None or weather_params.any() == None:
        weather_params = [0] * 11
    set_current_weather(weather_params)
    take_off(drone_pose, takeoff_height)
    return marker_pose

def set_obj_pose(obj_name, position, orientation):
    new_pose =airsim.Pose(position_val=airsim.Vector3r(position[0], position[1], position[2]),
     orientation_val=airsim.Quaternionr(orientation[0], orientation[1], orientation[2]))
    client.simSetObjectPose(obj_name, new_pose) 

def init_drone(drone_name='Copter', marker_pose=None):


    position = airsim.Vector3r(0, 0, -1.5)


    angle = random.uniform(0, 3.14)
    q1 = Quaternion(axis=[0., 0., 1.], angle=angle)

    orientation = airsim.Quaternionr(w_val=q1.elements[0], x_val=q1.elements[1], y_val=q1.elements[2], z_val=q1.elements[3])

    pose = airsim.Pose(position_val=position, orientation_val=orientation)
    client.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=drone_name)
    print('drone pose: ', pose)
    return pose

# set_obj_pose('DecalActor41', (0, 11, 0), (0, 0, 0, 1))

def restart(drone_name='Copter'):
    position = airsim.Vector3r(0, 0, -1)
    orientation = airsim.Quaternionr(0, 0, 0, 1)
    pose = airsim.Pose(position_val=position, orientation_val=orientation)
    client.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=drone_name)
    time.sleep(5)
    # marker_pose = respawn_marker('Cube_marker')

    # return marker_pose
 

def respawn_drone(drone_name='Copter', height=None, marker_pose=None):

    if not marker_pose:
        position = airsim.Vector3r(0, 0, -1.5)
    else:
        if not height:
            position = airsim.Vector3r(marker_pose.position.x_val + random.uniform(-1.5, 1.5),
            marker_pose.position.y_val + random.uniform(-3, 3), -1.5)
        else:
            position = airsim.Vector3r(marker_pose.position.x_val + random.uniform(-1.5, 1.5),
            marker_pose.position.y_val + random.uniform(-3, 3), height)            
    # orientation = airsim.Quaternionr(random.random(), random.random(), random.random(), random.random())

    angle = random.uniform(0, 3.14)
    q1 = Quaternion(axis=[0., 0., 1.], angle=angle)

    orientation = airsim.Quaternionr(w_val=q1.elements[0], x_val=q1.elements[1], y_val=q1.elements[2], z_val=q1.elements[3])
    # orientation = airsim.Quaternionr(w_val=0.7, x_val=0, y_val=0, z_val=0.7)
    # orientation = airsim.Quaternionr(w_val=1, x_val=0, y_val=0, z_val=0)

    pose = airsim.Pose(position_val=position, orientation_val=orientation)
    client.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=drone_name)
    # print('drone pose: ', pose)
    return pose

def destroy_mat(mat_name=''):
    result = client.simDestroyObject(mat_name)
    return result

def respawn_mat(mat_name='Leaf_02_Leaf_02_Dw', call_name = 'leaf1', x=0, y=0):
    true_marker_pose = client.simGetObjectPose('Cube_marker')
    # black_marker_pose = client.simGetObjectPose('non_marker_1')
    # white_marker_pose = client.simGetObjectPose('non_marker_3')
    # true_marker_pose.x_val = 0.707
    # true_marker_pose.w_val = 0.707

    position = true_marker_pose.position
    # position.x_val = location[0] + x
    # position.y_val = location[1]+ y
    position.x_val = position.x_val + x
    position.y_val = position.y_val + y




    call_name = client.simSpawnObject(object_name=call_name,asset_name=mat_name,pose=true_marker_pose, scale=Vector3r(2,2,4), physics_enabled=True)
    # leaf2 = client.simSpawnObject(object_name='leaf2',asset_name='Leaf_02_Leaf_02_Dw',pose=true_marker_pose, scale=Vector3r(0.1,0.1,0.1), physics_enabled=True)
    # a = client.simSpawnObject(object_name='Autumn_Leaves_tm',asset_name='Autumn_Leaves',pose=true_marker_pose, scale=Vector3r(0.1,0.1,0.1), physics_enabled=True)
    # a = client.simSpawnObject(object_name='Autumn_Leaves_tm',asset_name='Autumn_Leaves',pose=true_marker_pose, scale=Vector3r(0.1,0.1,0.1), physics_enabled=True)


    return call_name

def respawn_marker(marker_name='DecalActor41', drone_pose=None):

    if not drone_pose:
        # position = airsim.Vector3r(0 + random.random() * 2, random.randint(0, 20), 1)
        position = airsim.Vector3r(0, 2, 1)

    else:
        position = drone_pose.position
        position.z = 1
        # position = airsim.Vector3r(drone_pose.position.x_val + random.uniform(-2.5, 2.5), drone_pose.position.y_val + random.uniform(-2.5, 2.5), -1.5)
    orientation = airsim.Quaternionr(w_val=0.7, x_val=0, y_val=-0.7, z_val=0)
    # orientation = airsim.Quaternionr(random.random(), random.random(), random.random(), random.random())
    
    # position = airsim.Vector3r(0, 13, 1)
    pose = airsim.Pose(position_val=position, orientation_val=orientation)
    # print('position',position)
    # print('orientation',orientation)
    client.simSetObjectPose(marker_name, pose)
    print('marker pose: ', pose)
    return pose


def reset_markers():
    orientation = airsim.Quaternionr(w_val=0.7, x_val=0, y_val=-0.7, z_val=0)
    # orientation = airsim.Quaternionr(random.random(), random.random(), random.random(), random.random())
    
    position = airsim.Vector3r(0, 0, -3)
    pose = airsim.Pose(position_val=position, orientation_val=orientation)
    for i in range(50):
        client.simSetObjectPose('marker{}'.format(i), pose)
    for i in range(1, 4):
        client.simSetObjectPose('non_marker_{}'.format(i), pose)
    
    client.simSetObjectPose('marker_blur', pose)
    client.simSetObjectPose('marker_brightness_1', pose)
    client.simSetObjectPose('marker_brightness_2', pose)
    client.simSetObjectPose('marker_contrast', pose)
    client.simSetObjectPose('marker_shear', pose)
    client.simSetObjectPose('marker_translation', pose)    

def reset(marker_name='Marker_single'):
    orientation = airsim.Quaternionr(w_val=0.7, x_val=0, y_val=-0.7, z_val=0)
    # orientation = airsim.Quaternionr(random.random(), random.random(), random.random(), random.random())
    
    position = airsim.Vector3r(0, 0, -3)
    pose = airsim.Pose(position_val=position, orientation_val=orientation)
    for i in range(50):
        client.simSetObjectPose('marker{}'.format(i), pose)
    for i in range(1, 4):
        client.simSetObjectPose('non_marker_{}'.format(i), pose)
    
    client.simSetObjectPose('marker_blur', pose)
    client.simSetObjectPose('marker_brightness_1', pose)
    client.simSetObjectPose('marker_brightness_2', pose)
    client.simSetObjectPose('marker_contrast', pose)
    client.simSetObjectPose('marker_shear', pose)
    client.simSetObjectPose('marker_translation', pose)
    

    
    # current_marker = marker_id
    # current_marker = 43
    if marker_name:
        marker_pose = respawn_marker(marker_name=marker_name)
        drone_pose = respawn_drone(marker_pose=marker_pose)
    else:
        drone_pose = respawn_drone()


    set_current_weather([0] * 11)
    take_off(drone_pose)

    img = get_current_scene()

    return img


def init():

    position = airsim.Vector3r(0, 0, 1)
        # position = airsim.Vector3r(drone_pose.position.x_val + random.uniform(-2.5, 2.5), drone_pose.position.y_val + random.uniform(-2.5, 2.5), -1.5)
    orientation = airsim.Quaternionr(w_val=0.7, x_val=0, y_val=-0.7, z_val=0)
    # orientation = airsim.Quaternionr(random.random(), random.random(), random.random(), random.random())
    # position = airsim.Vector3r(0, 13, 1)
    pose = airsim.Pose(position_val=position, orientation_val=orientation)
    client.simSetObjectPose('marker0', pose)
    drone_pose = respawn_drone(marker_pose=pose)
    # offset = random.random() * 3
    offset = 0
    print('height: ', 15 + offset)
    take_off(drone_pose, height=15 + offset)


def respawn(marker_name='Marker_single'):
    # marker_pose = respawn_marker(marker_name='Marker_single')
    marker_pose = respawn_marker(marker_name=marker_name)
    # print(client.simSetObjectMaterial('Marker_single', 'marker_2_Mat'))
    # time.sleep(3)
    drone_pose = respawn_drone(marker_pose=marker_pose)


    # drone_pose = client.simGetVehiclePose('Copter')
    offset = random.random() * 3
    print('height: ', 10 + offset)
    take_off(drone_pose, height=15 + offset)

    # respawn_drone()
    # respawn_marker()

def get_obj_pose(obj_name):
    if obj_name != 'Copter':
        return client.simGetObjectPose(obj_name)
    else:
        return client.simGetVehiclePose(obj_name)
# weather_params = [0, 0, 0, 0, 0, 0, 0, 0]

def take_off(drone_pose, height=15):
    client.armDisarm(True)
    client.takeoffAsync().join()
    # height = 10 + 20 * random.random()
    move_to((drone_pose.position.x_val, drone_pose.position.y_val, height))

def land():
    client.landAsync().join()

# Async methods returns Future. Call join() to wait for task to complete.
def move_to(position):
    client.moveToPositionAsync(position[0], position[1], -position[2], 5).join()



def set_env_params(params):
    drone_pose = client.simGetVehiclePose('Copter')
    print(drone_pose.position)
    height = 5 + 20 * params[6]
    take_off(drone_pose, height)

    # time.sleep(5)
    #  [rain, snow, maple leaf, dust, fog, daytime, drone height]
    # client.simSetWeatherParameter(airsim.WeatherParameter.Rain, min(1, params[0]))
    client.simSetWeatherParameter(airsim.WeatherParameter.Roadwetness, min(1, params[0]))
    # client.simSetWeatherParameter(airsim.WeatherParameter.Snow, min(1, params[1]))
    client.simSetWeatherParameter(airsim.WeatherParameter.RoadSnow, min(1, params[1]))
    # client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, min(1, params[2]))
    client.simSetWeatherParameter(airsim.WeatherParameter.RoadLeaf, min(1, params[2]))
    client.simSetWeatherParameter(airsim.WeatherParameter.Dust, min(1, params[3]))
    client.simSetWeatherParameter(airsim.WeatherParameter.Fog, min(1, params[4]))
    # change light
    # intensity = params[5] * 5 # 0-5
    # print(client.simSetLightIntensity('Light Source', intensity))  
    day_time = 12 + int(params[5] * 12) # set the time range [12, 24]
    if day_time > 24:
        day_time = 24
    
    if day_time < 0:
        day_time = 0
    if day_time > 10:
        client.simSetTimeOfDay(is_enabled=True, start_datetime='2022-11-03 {}:00:00'.format(day_time),
         is_start_datetime_dst=True,  move_sun=True)
    else:
        client.simSetTimeOfDay(is_enabled=True, start_datetime='2022-11-03 0{}:00:00'.format(day_time),
         is_start_datetime_dst=True,  move_sun=True)

    time.sleep(3)
    fly_to(0, 50, -height)


def landing():
    client.landAsync(timeout_sec = 100, vehicle_name = 'Copter')

def set_current_weather(params):
    # Rain = 0
    # Roadwetness = 1
    # Snow = 2
    # RoadSnow = 3
    # MapleLeaf = 4
    # RoadLeaf = 5
    # Dust = 6
    # Fog = 7
    client.simSetWeatherParameter(airsim.WeatherParameter.Rain, min(1, params[0]))
    client.simSetWeatherParameter(airsim.WeatherParameter.Roadwetness, min(1, params[1]))
    client.simSetWeatherParameter(airsim.WeatherParameter.Snow, min(1, params[2]))
    client.simSetWeatherParameter(airsim.WeatherParameter.RoadSnow, min(1, params[3]))
    client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, min(1, params[4]))
    client.simSetWeatherParameter(airsim.WeatherParameter.RoadLeaf, min(1, params[5]))
    client.simSetWeatherParameter(airsim.WeatherParameter.Dust, min(1, params[6]))
    client.simSetWeatherParameter(airsim.WeatherParameter.Fog, min(1, params[7]))

    wind = airsim.Vector3r(min(5, params[8]*5), min(5, params[8]*5), 0)
    client.simSetWind(wind)

    # change light  
    day_time = 12 + int(params[9] * 12) # set the time range [12, 24]
    if day_time > 24:
        day_time = 24
    
    if day_time < 0:
        day_time = 0
    if day_time > 10:
        client.simSetTimeOfDay(is_enabled=True, start_datetime='2022-11-03 {}:00:00'.format(day_time),
         is_start_datetime_dst=True,  move_sun=True)
    else:
        client.simSetTimeOfDay(is_enabled=True, start_datetime='2022-11-03 0{}:00:00'.format(day_time),
         is_start_datetime_dst=True,  move_sun=True)

def get_current_scene(epoch=0, image_type=0):
    if image_type == 0:
        responses = client.simGetImages([airsim.ImageRequest("downward_custom", image_type, False, False)])
    elif image_type == 5:
        responses = client.simGetImages([airsim.ImageRequest("downward_semantic", image_type, False, False)])

    response = responses[0]
    # get numpy array
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

    # reshape array to 4 channel image array H X W X 4
    img_rgb = img1d.reshape(response.height, response.width, 3)

    # original image is fliped vertically
    # img_rgb = np.flipud(img_rgb)

    # write to png 
    # airsim.write_png(os.path.normpath('images/scene/current_scene_{}.png'.format(epoch)), img_rgb)
    
    return img_rgb

def setnpccarpos(object_name):
    car_pose = client.simGetObjectPose(object_name)
    # print(car_pose)

    # car_pose.orientation.w_val = 0.542
    # car_pose.orientation.x_val = 0.542
    # car_pose.orientation.y_val = 0.455
    # car_pose.orientation.z_val = -0.455


    car_pose.position.x_val =0
    car_pose.position.y_val =50
    car_pose.position.z_val =0

    a = client.simAddVehicle(vehicle_name='newcar', vehicle_type= "simpleflight", pose= car_pose)

    return a


def carmove(object_name):
    car_pose = client.simGetObjectPose(object_name)
    # print(car_pose)

    # car_pose.orientation.w_val = 0.542
    # car_pose.orientation.x_val = 0.542
    # car_pose.orientation.y_val = 0.455
    # car_pose.orientation.z_val = -0.455


    car_pose.position.x_val =0
    car_pose.position.y_val =0
    car_pose.position.z_val =0

    a = client.simSetObjectPose(object_name, car_pose, teleport = False)

    return a

if __name__ == "__main__":
    respawn()
    fly_to(0, -30, -10)
    # add_car(0, 3, 0)


# print(q1)
# print()
# drone_pose = client.simGetVehiclePose('Copter')

# take_off(drone_pose)
# print(client.simGetVehiclePose('Copter'))
# print(client.simGetCameraInfo('downward_custom'))
# marker_pose = client.simGetObjectPose('DecalActor41')
# client.simPlotPoints([marker_pose.position,
#  airsim.Vector3r(marker_pose.position.x_val + 0.5, marker_pose.position.y_val + 0.5, marker_pose.position.z_val),
#  airsim.Vector3r(marker_pose.position.x_val - 0.5, marker_pose.position.y_val - 0.5, marker_pose.position.z_val)], is_persistent=True)
# client.simSetLightIntensity('LightSource', 10)

# client.simSetTimeOfDay(is_enabled=True, start_datetime='2022-11-03 10:40:00', is_start_datetime_dst=True,  move_sun=True)


# get_current_scene(9999, 0)

# respawn_marker()
# respawn_drone()