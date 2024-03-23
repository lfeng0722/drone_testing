from simulation.msg import Marker as MarkerMSG
from simulation.msg import Weather as WeatherMSG
from simulation.msg import ScenarioPose as PoseMSG
from simulation.msg import TimeOfDay as TimeMSG
from simulation.msg import Actor as ActorMSG
from airsim_simulation.configuration import MARKER_MATERIAL_DICT, ACTOR_TYPE_DICT, ACTOR_TYPE_DICT_INV
import random

class Pose:
    def __init__(self, x, y, z, angle=0):
        self.x = x
        self.y = y
        self.z = z
        self.angle = angle

    def to_dict(self):
        if self.angle is not None:
            return {'pos_x': self.x, 'pos_y': self.y, 'pos_z': self.z, 'angle': self.angle}
        return {'pos_x': self.x, 'pos_y': self.y, 'pos_z': self.z}

    def to_vec(self):
        return [self.x, self.y, self.z, self.angle]
    
    def to_msg(self):
        msg = PoseMSG()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.yaw = self.angle
        return msg

    @classmethod
    def from_dict(cls, data):
        return cls(data['pos_x'], data['pos_y'], data['pos_z'], data.get('angle'))
    
    @classmethod
    def from_vec(cls, data):
        return cls(*data)
    
    @classmethod
    def from_msg(cls, data):
        return cls(data.x, data.y, data.z, data.yaw)
    
    @classmethod
    def crossover(cls, pose_1, pose_2):
        pose_1_vec = pose_1.to_vec()
        pose_2_vec = pose_2.to_vec()
        half_len = len(pose_1_vec) // 2
        new_pose_vec_1 = pose_1_vec[:half_len] + pose_2_vec[half_len:]
        new_pose_vec_2 = pose_2_vec[:half_len] + pose_1_vec[half_len:]
        return cls.from_vec(new_pose_vec_1), cls.from_vec(new_pose_vec_2)
    
    def mutate(self, mutation_rate=0.3):
        flag = random.random()
        if flag <= mutation_rate:
            self.x += random.uniform(-3, 3)
            self.y += random.uniform(-3, 3)
            self.angle += random.uniform(0, 90)
            self.angle %= 360

class Time:
    def __init__(self, hour, minute):
        self.hour = hour
        self.minute = minute

    def to_dict(self):
        return {'hour': self.hour, 'minute': self.minute}
    
    def to_vec(self):
        return [self.hour, self.minute]
    
    def to_msg(self):
        msg = TimeMSG()
        msg.hour = self.hour
        msg.minute = self.minute
        return msg

    @classmethod
    def from_dict(cls, data):
        return cls(data['hour'], data['minute'])
    
    @classmethod    
    def from_vec(cls, data):
        return cls(*data)
    
    @classmethod
    def from_msg(cls, data):
        return cls(data.hour, data.minute)
    
    @classmethod
    def crossover(cls, time_1, time_2):
        return cls(time_1.hour, time_2.minute), cls(time_2.hour, time_1.minute)
    
    def mutate(self, mutation_rate=0.3):
        flag = random.random()
        if flag <= mutation_rate:
            self.hour += random.uniform(-0.1, 0.1)
            self.minute += random.uniform(-0.1, 0.1)
            self.hour = max(0, min(1, self.hour))
            self.minute = max(0, min(1, self.minute))

class Weather:
    def __init__(self, rain, road_wetness, snow, road_snow, maple_leaf, road_leaf, dust, fog, wind):
        self.rain = 0
        self.road_wetness = road_wetness
        self.snow = 0
        self.road_snow = road_snow
        self.maple_leaf = 0
        self.road_leaf = road_leaf
        self.dust = 0
        self.fog = fog
        self.wind = wind

    def to_vec(self):
        return [self.rain, self.road_wetness, self.snow, self.road_snow, self.maple_leaf,
         self.road_leaf, self.dust, self.fog, self.wind]
        
    def to_dict(self):
        return {
            'rain': self.rain,
            'road_wetness': self.road_wetness,
            'snow': self.snow,
            'road_snow': self.road_snow,
            'maple_leaf': self.maple_leaf,
            'road_leaf': self.road_leaf,
            'dust': self.dust,
            'fog': self.fog,
            'wind': self.wind
        }
    
    def to_msg(self):
        msg = WeatherMSG
        msg.rain = self.rain
        msg.road_wetness = self.road_wetness
        msg.road_snow = self.road_snow
        msg.maple_leaf = self.maple_leaf
        msg.road_leaf = self.road_leaf
        msg.dust = self.dust
        msg.fog = self.fog
        msg.wind = self.wind
        return msg

    @classmethod
    def from_dict(cls, data):
        return cls(data['rain'], data['road_wetness'], data['snow'], data['road_snow'], 
                   data['maple_leaf'], data['road_leaf'], data['dust'], data['fog'], data['wind'])
    
    @classmethod
    def from_vec(cls, data):
        return cls(*data)  
    
    @classmethod
    def from_msg(cls, data):
        return cls(data.rain, data.road_wetness, data.snow, data.road_snow, data.maple_leaf,
                   data.road_leaf, data.dust, data.fog, data.wind)   
    
    @classmethod
    def crossover(cls, weather_1, weather_2):
        weather_1_vec = weather_1.to_vec()
        weather_2_vec = weather_2.to_vec()
        half_len = len(weather_1_vec) // 2
        new_weather_vec_1 = weather_1_vec[:half_len] + weather_2_vec[half_len:]
        new_weather_vec_2 = weather_2_vec[:half_len] + weather_1_vec[half_len:]

        return cls.from_vec(new_weather_vec_1), cls.from_vec(new_weather_vec_2)
    
    def mutate(self, mutation_rate=0.3):
        flag = random.random()
        if flag <= mutation_rate:
            self.rain += random.uniform(-0.1, 0.1)
            self.road_wetness += random.uniform(-0.1, 0.1)
            self.snow += random.uniform(-0.1, 0.1)
            self.road_snow += random.uniform(-0.1, 0.1)
            self.maple_leaf += random.uniform(-0.1, 0.1)
            self.road_leaf += random.uniform(-0.1, 0.1)
            self.dust += random.uniform(-0.1, 0.1)
            self.fog += random.uniform(-0.1, 0.1)
            self.wind += random.uniform(-0.1, 0.1)

            self.rain = max(min(self.rain, 0.5), 0)
            self.road_wetness = max(min(self.road_wetness, 0.5), 0)
            self.snow = max(min(self.snow, 0.5), 0)
            self.road_snow = max(min(self.road_snow, 0.5), 0)
            self.maple_leaf = max(min(self.maple_leaf, 0.5), 0)
            self.road_leaf = max(min(self.road_leaf, 0.5), 0)
            self.dust = max(min(self.dust, 0.5), 0)
            self.fog = max(min(self.fog, 0.5), 0)
            self.wind = max(min(self.wind, 0.5), 0)

class Marker(object):
    def __init__(self, id, material, pose):
        self.id = id
        self.material = material
        self.pose = pose

    def to_vec(self):
        return [self.id, self.material] + self.pose.to_vec()

    def to_dict(self):
        marker_dict = {'id': self.id, 'material': self.material}
        marker_dict.update(self.pose.to_dict())
        return marker_dict
    
    def to_msg(self):
        msg = MarkerMSG()
        msg.id = self.id
        msg.material = self.material

        msg.pos_x = self.pose.x
        msg.pos_y = self.pose.y
        msg.pos_z = self.pose.z
        msg.yaw_angle = self.pose.angle
        return msg

    @classmethod
    def from_dict(cls, data):
        pose = Pose.from_dict(data)
        return cls(data['id'], data['material'], pose)
    
    @classmethod
    def from_vec(cls, data):
        pose = Pose.from_vec(data[2:])
        return cls(data[0], data[1], pose)
    
    @classmethod
    def from_msg(cls, data):
        pose = Pose(data.pos_x, data.pos_y, data.pos_z, data.yaw_angle)
        return cls(data.id, data.material, pose)
    
    @classmethod
    def crossover(cls, marker_1, marker_2):
        marker_1_vec = marker_1.to_vec()
        marker_2_vec = marker_2.to_vec()
        half_len = len(marker_1_vec) // 2
        new_marker_vec_1 = marker_1_vec[:half_len] + marker_2_vec[half_len:]
        new_marker_vec_2 = marker_2_vec[:half_len] + marker_1_vec[half_len:]

        return cls.from_vec(new_marker_vec_1), cls.from_vec(new_marker_vec_2)
    
    def mutate(self, mutation_rate=0.3, mutate_id=False):
        # Introduce small random changes to the position and angle of the marker
        self.pose.mutate(mutation_rate)
        flag = random.random()
        if flag <= mutation_rate:
            if mutate_id:
                self.id = random.choice([2, 3, 4, 5, 6, 7])
        # self.pos_z += random.uniform(-1, 1)  # Change z by [-1, 1] units
        # self.yaw_angle += random.uniform(-0.1, 0.1)  # Change yaw by [-0.1, 0.1] radians

class Actor(object):
    def __init__(self, type, start_pose, end_pose, speed):
        self.type = type
        self.start_pose = start_pose
        self.end_pose = end_pose
        self.speed = speed

    def to_vec(self):
        return [self.type] + self.start_pose.to_vec() + self.end_pose.to_vec() + [self.speed]

    def to_dict(self):
        actor_dict = {'type': ACTOR_TYPE_DICT[self.type]}
        actor_dict['start_pose'] = self.start_pose.to_dict()
        actor_dict['end_pose'] = self.end_pose.to_dict()
        actor_dict['speed'] = self.speed
        return actor_dict

    def to_msg(self):
        msg = ActorMSG()
        msg.type = self.type
        start_pose_msg = self.start_pose.to_msg()
        end_pose_msg = self.end_pose.to_msg()
        end_pose_msg.yaw = self.end_pose.angle

        msg.start_pose = start_pose_msg
        msg.end_pose = end_pose_msg
        msg.speed = self.speed
        return msg

    @classmethod
    def from_dict(cls, data):
        start_pose = Pose.from_dict(data['start_pose'])
        end_pose = Pose.from_dict(data['end_pose'])
        return cls(data['type'], start_pose, end_pose, data['speed'])

    @classmethod
    def from_vec(cls, data):
        start_pose = Pose.from_vec(data[1:5])
        end_pose = Pose.from_vec(data[5:9])
        return cls(data[0], start_pose, end_pose, data[-1])    
    
    @classmethod
    def from_msg(cls, data):
        start_pose = Pose.from_msg(data.start_pose)
        end_pose = Pose.from_msg(data.end_pose)
        return cls(ACTOR_TYPE_DICT_INV[data.type], start_pose, end_pose, data.speed)
    
    @classmethod
    def crossover(cls, actor_1, actor_2):
        actor_1_vec = actor_1.to_vec()
        actor_2_vec = actor_2.to_vec()
        half_len = len(actor_1_vec) // 2
        new_actor_vec_1 = actor_1_vec[:half_len] + actor_2_vec[half_len:]
        new_actor_vec_2 = actor_2_vec[:half_len] + actor_1_vec[half_len:]

        return cls.from_vec(new_actor_vec_1), cls.from_vec(new_actor_vec_1)
    
    def mutate(self, mutation_rate=0.3):
        self.start_pose.mutate(mutation_rate)  # Change x by [-1, 1] units
        self.end_pose.mutate(mutation_rate) # Change y by [-1, 1] units
        flag = random.random()
        if flag < mutation_rate:
            self.type = random.choice(list(ACTOR_TYPE_DICT.keys()))
            if ACTOR_TYPE_DICT[self.type] == 'bird':
                self.start_pose.z = random.uniform(3, 8)
                self.end_pose.z = self.start_pose.z
        flag = random.random()
        if flag < mutation_rate:        
            self.speed = random.random()