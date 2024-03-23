class DroneController:
    def __init__(self, drone_name):
        self.drone_name = drone_name
        self.drone_status = None
    
    def takeoff(self, height, speed):
        pass

    def move_to_local(self, pose, speed):
        pass

    def move_to_global(self, pose, speed):
        pass

    def land(self):
        pass