from control.drone_control import DroneController
import airsim
import math
import time

class AirSimController(DroneController):
    def __init__(self, host_ip, drone_name='Copter'):
        self.airsim_client = airsim.MultirotorClient(host_ip)
        self.airsim_client.confirmConnection()
        self.airsim_client.enableApiControl(True)
        self.drone_name = drone_name
    
    def move_to_global(self, scenario_pose, velocity=3):
        self.airsim_client.moveToPositionAsync(scenario_pose.x, scenario_pose.y, scenario_pose.z, velocity)
    
    def move_to_local(self, scenario_pose, velocity=(1, 1, 1), duration=0.2):
        vx_max = velocity.y
        vy_max = velocity.x
        z = scenario_pose.z
        duration = duration
        # print(vx_max, vy_max, z, duration)
        self.airsim_client.moveByVelocityZAsync(vx_max, vy_max, z, duration=0.001)
    
    def takeoff(self, height=15, velocity=3):
        # self.land()
        self.airsim_client.enableApiControl(True)
        self.airsim_client.armDisarm(True)
        time.sleep(2)
        # self.airsim_client.takeoffAsync().join()
        current_pose = self.airsim_client.simGetVehiclePose(self.drone_name)
        new_position = airsim.Vector3r(0,  0, -height)
        new_pose = airsim.Pose(position_val=new_position)
        print('takeoff height: ', height)
        self.airsim_client.moveToPositionAsync(0, 0,
         -height, velocity).join()
        print('finish takeoff')

        # self.move_to_global(new_pose, velocity)
    
    def land(self):
        print('land')
        self.airsim_client.landAsync().join()
        self.airsim_client.armDisarm(False)
    
    def set_pose(self, pose):
        self.airsim_client.setVehiclePose(pose, self.drone_name)