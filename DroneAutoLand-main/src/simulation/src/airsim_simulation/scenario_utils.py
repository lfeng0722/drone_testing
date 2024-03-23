try:
    from airsim_simulation.components import  Marker, Pose, Time, Weather, Actor
    # from airsim_simulation.scenario import Scenario
    from airsim_simulation.configuration import MARKER_MATERIAL_DICT, ACTOR_TYPE_DICT
except: 
    from simulation.src.airsim_simulation.components import  Marker, Pose, Time, Weather, Actor
    # from simulation.src.airsim_simulation.scenario import Scenario
    from simulation.src.airsim_simulation.configuration import MARKER_MATERIAL_DICT, ACTOR_TYPE_DICT  

import random
import math


def sample_marker(marker_pose, type='tp'):
    if type == 'tp':
        # random_id = random.randint(0, 3)
        random_id = 0
    else:
        random_id = random.randint(1, 2)
    random_material = MARKER_MATERIAL_DICT[0]

    marker = Marker(random_id, random_material, marker_pose)
    return marker

def sample_pose(range_x, range_y):
    # if map_name == 'map_0':
    #     random_pos_x = random.uniform(-10, 10)  # Just an example range
    #     random_pos_y = random.uniform(-35, 35)
    # else:
    #     random_pos_x = random.uniform(-8, 8)  # Just an example range
    #     random_pos_y = random.uniform(-50, 50)
    random_pos_x = random.uniform(range_x[0], range_x[1])
    random_pos_y = random.uniform(range_y[0], range_y[1])
    pos_z = 0.01
    # if ACTOR_TYPE_DICT[type] != 'bird':
    #     random_pos_z = 0.5
    # else:
    #     random_pos_z = random.uniform(5, 10)
    random_yaw_angle = random.uniform(0, 360)
    pose = Pose(random_pos_x, random_pos_y, pos_z, random_yaw_angle)
    return pose


def sample_pose_on_circle(gps_pose, radius):
    # Get a random radius between 0 and the circle's radius
    r = random.uniform(0, radius)

    # Get a random angle between 0 and 2π
    theta = random.uniform(0, 2 * math.pi)

    # Convert from polar to Cartesian coordinates
    x_offset = r * math.cos(theta)
    y_offset = r * math.sin(theta)    
    # Add offsets to gps_pose to get marker's position
    x = gps_pose.x + x_offset
    y = gps_pose.y + y_offset
    # if actor_type == -1:
    #     z = gps_pose.z
    # elif 'bird' not in ACTOR_TYPE_DICT[actor_type]:
    #     z = 0.5
    # else:
    #     z = random.uniform(5, 10)

    # random_yaw_angle = random.uniform(0, 360)

    return [x, y, 0]

def distance_2d(point1, point2):
    """Calculate the Euclidean distance between two points in 2D space."""
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def sample_marker_in_circle(gps_pose, radius, type='tp'):
    """
    Given a gps_pose [x, y, z] and a radius, randomly sample the position of a marker 
    inside the circular region where the circle's center is the gps_pose.

    :param gps_pose: List containing [x, y, z] coordinates of the circle's center.
    :param radius: Radius of the circle.
    :return: A list containing [x, y, z] coordinates of the sampled marker.
    """

    marker_x, marker_y, marker_z = sample_pose_on_circle(gps_pose, radius)

    # random_yaw_angle = random.uniform(0, 360)
    marker_pose = Pose(marker_x, marker_y, marker_z, 0) # current fix the angle of the marker
    marker = sample_marker(marker_pose, type)
    return marker

def sample_actor(gps_pose, radius=5, dynamic=False):
    actor_type = random.choice(list(ACTOR_TYPE_DICT.keys()))
    start_x, start_y, start_z = sample_pose_on_circle(gps_pose, radius)
    start_pose = Pose(start_x, start_y, start_z, 0)
    if not dynamic:
        actor = Actor(actor_type, start_pose, start_pose, 0)
    else:
        end_x = start_x + random.uniform(-10, 10)
        end_y = start_y + random.uniform(-10, 10)
        end_z = start_z
        end_pose = Pose(end_x, end_y, end_z, 0)
        speed = random.random()
        actor = Actor(actor_type, start_pose, end_pose, speed)
    return actor


    
