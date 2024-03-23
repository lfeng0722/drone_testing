import airsim
from simulation.msg import ScenarioPose

class GlobalDecisionMaker:
    def __init__(self, search_height=15):
        self.search_height = search_height

    def generate_global_trajectory(self, scenario):
        target_pose = scenario.gps_pose
        trajectory = []
        pose = ScenarioPose()
        pose.x = target_pose.x
        pose.y = target_pose.y
        pose.z = -self.search_height
        pose.yaw = 0
        # pose = airsim.Pose(airsim.Vector3r(target_pose.x, target_pose.y, -self.search_height))
        trajectory.append(pose)
        return trajectory