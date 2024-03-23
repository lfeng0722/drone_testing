import json
import random
from airsim_simulation.components import Marker, Actor, Weather, Time, Pose
from simulation.msg import Scenario as ScenarioMSG
from airsim_simulation.scenario_utils import sample_actor, sample_marker, sample_marker_in_circle, sample_pose, sample_pose_on_circle, distance_2d
try:
    from airsim_simulation.configuration import MARKER_MATERIAL_DICT, ACTOR_TYPE_DICT
except: 
    from simulation.src.airsim_simulation.configuration import MARKER_MATERIAL_DICT, ACTOR_TYPE_DICT  


def sample_test_scenario():
    gps_pose = Pose(0, 10, -1, 0)
    radius = 2
    tp_marker = sample_marker_in_circle(gps_pose, radius, 'tp')
    tp_marker.id = 0
    fp_markers= []
    drone_start_pose = Pose(0, 0, 0, 0)
    weather = Weather(*([0] * 9))
    time = Time(0.5, 0.5)
    actors = []
    for _ in range(1):
        start_x, start_y, start_z = sample_pose_on_circle(gps_pose, 2, 0)
        pose = Pose(start_x, start_y, start_z, 0)
        actors.append(Actor(0, pose, pose, 0))

    scenario = Scenario()
    scenario.init(tp_marker, fp_markers, drone_start_pose, gps_pose, radius, actors, weather, time)    
    return scenario

def sample_scenario():

    # first define the gps pose and the radius
    gps_pose = sample_pose()
    radius = 7.5

    all_marker_pos = [(0, 0)]
    # Sample a random tp_marker inside the circular region with the center gps_pos and the radius.
    while True:
        new_marker = sample_marker_in_circle(gps_pose, radius, 'tp')
        new_marker_pos = [new_marker.pose.x, new_marker.pose.y]
        if all(distance_2d(new_marker_pos, marker_pos) > 2 for marker_pos in all_marker_pos):
            all_marker_pos.append(new_marker_pos)
            tp_marker = new_marker
            break
    # tp_marker = sample_marker_in_circle(gps_pose, radius, 'tp')
    # all_marker_pos.append([tp_marker.pose.x, tp_marker.pose.y])
    # Sample a random list of fp_markers 
    fp_markers = []
    # num_fp_markers = random.randint(0, 3)  # For example, choose up to 10 fp_markers
    num_fp_markers = 2
    # add valid markers that do not overlap
    while len(all_marker_pos) < num_fp_markers + 2:
        new_marker = sample_marker_in_circle(gps_pose, radius, 'fp')
        new_marker_pos = [new_marker.pose.x, new_marker.pose.y]
        if all(distance_2d(new_marker_pos, marker_pos) > 2 for marker_pos in all_marker_pos):
            new_marker.id = len(fp_markers) + 1
            fp_markers.append(new_marker)
            all_marker_pos.append(new_marker_pos)

    # Sample random drone_start_pose
    drone_radius = radius + random.uniform(10, 30)
    drone_start_x, drone_start_y, drone_start_z = sample_pose_on_circle(gps_pose, drone_radius)
    random_yaw_angle = random.uniform(0, 360)
    drone_start_pose = Pose(drone_start_x, drone_start_y, drone_start_z, random_yaw_angle)


    # Sample a random list of actors
    actors = []
    all_actor_pos = []
    num_actors = random.randint(1, 3)  # For example, choose up to 5 actors
    print('sample number of actors: ', num_actors)
    # for _ in range(num_actors):
    while len(actors) < num_actors:
        actor_type = random.choice(list(ACTOR_TYPE_DICT.keys()))
        actor_radius = radius + random.uniform(0, drone_radius)
        start_x, start_y, start_z = sample_pose_on_circle(gps_pose, actor_radius, actor_type)
        start_pos = (start_x, start_y)
        if all(distance_2d(start_pos, actor_pos) > 1 for actor_pos in all_actor_pos):
            start_angle = random.uniform(0, 360)
            # current set the actor as static
            # end_x, end_y, end_z = start_x, start_y, start_z
            end_x, end_y, _ = sample_pose_on_circle(gps_pose, actor_radius, actor_type)
            end_z = start_z

            end_angle = start_angle
            # speed = 0
            speed = random.uniform(0.2, 1)
            # speed = random.uniform(0, 10)  # Example speed range
            start_pose = Pose(start_x, start_y, start_z, start_angle)
            end_pose = Pose(end_x, end_y, end_z, end_angle)
            random_actor = Actor(actor_type, start_pose, end_pose, speed)
            all_actor_pos.append(start_pos)
            actors.append(random_actor)


    # Sample random weather
    weather = Weather(*[random.uniform(0, 0.15) for _ in range(9)])  # Assuming weather conditions are binary

    # Sample random time
    time = Time(*[random.uniform(0, 1), random.uniform(0, 1)])  # Random hour and minute
    scenario = Scenario()
    scenario.init(tp_marker, fp_markers, drone_start_pose, gps_pose, radius, actors, weather, time)

    return scenario



class Scenario(object):
    def __init__(self):
        self.tp_marker = None
        self.fp_markers = []
        self.drone_start_pose = None
        self.gps_pose = None
        self.radius = None # marker sample radius
        self.actors = []
        self.weather = None
        self.time = None
    

    def init(self, tp_marker, fp_markers, drone_start_pose, gps_pose, radius, actors, weather, time):
        self.tp_marker = tp_marker
        self.fp_markers = fp_markers
        self.drone_start_pose= drone_start_pose
        self.gps_pose = gps_pose
        self.radius = radius
        self.actors = actors
        self.weather = weather
        self.time = time

    def to_json(self, save_path=None):
        json_dict = {}
        json_dict['tp_marker'] = self.tp_marker.to_dict()
        json_dict['drone_start_pose'] = self.drone_start_pose.to_dict()
        json_dict['gps_pose'] = self.gps_pose.to_dict()
        json_dict['radius'] = self.radius   
        json_dict['weather'] = self.weather.to_dict()
        json_dict['time'] = self.time.to_dict()

        json_dict['fp_markers'] = [marker.to_dict() for marker in self.fp_markers]
        json_dict['actors'] = [actor.to_dict() for actor in self.actors]

        if save_path:
            with open(save_path, 'w') as json_file:
                json.dump(json_dict, json_file)

        return json_dict


    def to_vec_dict(self):
        vec_dict = {}
        vec_dict['tp_marker'] = self.tp_marker.to_vec()
        vec_dict['drone_start_pose'] = self.drone_start_pose.to_vec()
        vec_dict['gps_pose'] = self.gps_pose.to_vec()
        vec_dict['radius'] = self.radius
        vec_dict['weather'] = self.weather.to_vec()
        vec_dict['time'] = self.time.to_vec()

        vec_dict['fp_markers'] = [marker.to_vec() for marker in self.fp_markers]
        vec_dict['actors'] = [actor.to_vec() for actor in self.actors]
        
        return vec_dict
    
    def to_msg(self):
        tp_marker_msg = self.tp_marker.to_msg()
        drone_start_pose_msg = self.drone_start_pose.to_msg()
        gps_pose_msg = self.gps_pose.to_msg()
        weather_msg = self.weather.to_msg()
        time_msg = self.time.to_msg()

        fp_markers_msg = [marker.to_msg() for marker in self.fp_markers]
        actors_msg = [actor.to_msg() for actor in self.actors]
        msg = ScenarioMSG()
        msg.tp_marker = tp_marker_msg
        msg.drone_start_pose = drone_start_pose_msg
        msg.gps_pose = gps_pose_msg
        msg.weather = weather_msg
        msg.time = time_msg
        msg.fp_markers = fp_markers_msg
        msg.actos = actors_msg
        msg.status = 1

        return msg

    def load_from_vec_dict(self, vec_dict):
        self.tp_marker = Marker.from_vec(vec_dict['tp_marker'])
        self.drone_start_pose = Pose.from_vec(vec_dict['drone_start_pose'])
        
        self.gps_pose = Pose.from_dict(vec_dict['gps_pose'])

        self.radius = vec_dict['radius']

        self.weather = Weather.from_vec(vec_dict['weather'])
        self.time = Time.from_vec(vec_dict['time'])

        self.fp_markers = [Marker.from_vec(vec_data) for vec_data in vec_dict['fp_markers']]
        
        # Assuming the actor vector has 8 values: type, start_x, start_y, start_z, start_angle, end_x, end_y, end_z, speed
        self.actors = []
        for actor_vec in vec_dict['actors']:
            # actor_type = actor_vec[0]
            # start_pose = Pose(*actor_vec[1:5])
            # end_pose = Pose(*actor_vec[5:9])
            # speed = actor_vec[9]
            self.actors.append(Actor.from_vec(actor_vec))

    def load_from_json(self, json_path):
        if type(json_path) == str:
            with open(json_path, 'r') as f:
                json_dict = json.load(f)
        else:
            json_dict = json_path
        self.tp_marker = Marker.from_dict(json_dict['tp_marker'])
        self.drone_start_pose = Pose.from_dict(json_dict['drone_start_pose'])
        self.gps_pose = Pose.from_dict(json_dict['gps_pose'])
        self.radius = json_dict['radius']
        self.weather = Weather.from_dict(json_dict['weather'])
        self.time = Time.from_dict(json_dict['time'])

        self.fp_markers = [Marker.from_dict(marker_data) for marker_data in json_dict['fp_markers']]
        self.actors = [Actor.from_dict(actor_data) for actor_data in json_dict['actors']]      
    
    def load_from_msg(self, scenario_msg):
        self.tp_marker = Marker.from_msg(scenario_msg.tp_marker)
        self.drone_start_pose = Pose.from_msg(scenario_msg.drone_start_pose)
        self.gps_pose = Pose.from_msg(scenario_msg.gps_pose)
        self.radius = scenario_msg.radius
        self.weather = Weather.from_msg(scenario_msg.weather)
        self.time = Time.from_msg(scenario_msg.time)

        self.fp_markers = [Marker.from_msg(marker_msg) for marker_msg in scenario_msg.fp_markers]
        self.actors = [Actor.from_msg(actor_msg) for actor_msg in scenario_msg.actors]      

    def mutate(self, mutation_rate=0.3):
        # self.tp_marker.mutate(mutation_rate)
        self.drone_start_pose.mutate(mutation_rate)
        self.gps_pose.mutate(mutation_rate)
        self.tp_marker = sample_marker_in_circle(self.gps_pose, self.radius, 'tp')
        self.weather.mutate(mutation_rate)
        self.time.mutate(mutation_rate)
        flag = random.random()
        if flag <= mutation_rate:
            self.radius += random.uniform(-3, 3)

        for marker in self.fp_markers:
            marker.mutate(mutation_rate)
        
        # mutate actors
        flag = random.random() # whether mutate the number of actors
        if flag <= mutation_rate:
            num_actors = len(self.actors) + random.randint(-2, 2)
        else:
            num_actors = len(self.actors)

        if num_actors <= 0:
            self.actors = []
        elif num_actors <= len(self.actors):
            self.actors = self.actors[:num_actors]
            for i in range(num_actors):
                self.actors[i].mutate()
        else:
            for i in range(len(self.actors)):
                self.actors[i].mutate()
            
            for i in range(num_actors - len(self.actors)):
                self.actors.append(sample_actor(self.gps_pose))
        
        # check numbers of different types of actors
        # actor_types = {}



    @classmethod
    def crossover(cls, scenario_1, scenario_2):
        # tp_marker_crossover
        tp_marker_1, tp_marker_2 = Marker.crossover(scenario_1.tp_marker, scenario_1.tp_marker)
        drone_start_pose_1, drone_start_pose_2 = Pose.crossover(scenario_1.drone_start_pose, scenario_2.drone_start_pose)
        gps_pose_1, gps_pose_2 = Pose.crossover(scenario_1.gps_pose, scenario_2.gps_pose)
        radius_1, radius_2 = scenario_2.radius, scenario_1.radius
        weather_1, weather_2 = Weather.crossover(scenario_1.weather, scenario_2.weather)
        time_1, time_2 = Time.crossover(scenario_1.time, scenario_2.time)

        fp_markers_1, fp_markers_2 = scenario_2.fp_markers, scenario_1.fp_markers
        actors_1, actors_2 = scenario_2.actors, scenario_1.actors

        new_scenario_1 = Scenario()
        new_scenario_2 = Scenario()
        new_scenario_1.init(tp_marker_1, fp_markers_1, drone_start_pose_1, gps_pose_1, radius_1, actors_1, weather_1, time_1)
        new_scenario_2.init(tp_marker_2, fp_markers_2, drone_start_pose_2, gps_pose_2, radius_2, actors_2, weather_2, time_2)

        return new_scenario_1, new_scenario_2


    def __str__(self):
        return str(self.to_json())

