import cv2
from cv2 import aruco
import numpy
from airsim_api import *
import math
from yolo_detector import load_model, detect
from yolov5.detect import parse_opt


def fitness_func(model, opt, solution, mode='yolo'):
    # global solutions
    # print(solution)
    restart()
    set_env_params(solution)
    # time.sleep(5)
    drone_pose = get_obj_pose('Copter')
    drone_position = drone_pose.position
    drone_orientation = drone_pose.orientation
    # solution = np.array()
    # time.sleep(5)
    # solution = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    fitness = 0
    detect_count = 0
    for i in range(30):
        image = get_current_scene()

        if mode == 'opencv':
            arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
            arucoParams = cv2.aruco.DetectorParameters_create()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
                                                        parameters=arucoParams)
            image_debug = cv2.aruco.drawDetectedMarkers(image, corners, ids)
            cv2.imshow('Detection result', image_debug)
            k = cv2.waitKey(200)
            if k == ord('q') & 0xFF:
                break
            # print(solution_idx, ids)

            fitness1 = 1 / np.linalg.norm(np.array(solution)-np.zeros(11))
            # if not ids:
            #     fitness2 = 1
            # else:
            #     fitness2 = 0
            
            
            # fitness += fitness2

            try:
                fitness2 = 3 - len(ids)
            except :
                fitness2 = 3

            time.sleep(0.1)
        elif mode == 'yolo':
            # fitness1 = 1 / np.linalg.norm(np.array(solution)-np.zeros(11))
            detect_result, result_img = detect(model, image, opt)
            print(detect_result)
            if len(detect_result[0]) != 0:
                for result in detect_result[0]:
                    obj_position_c = pixel_to_pos(drone_position.z_val, result)
                    # print('obj_position_c: ', obj_position_c)
                    obj_position_g = coord_convert(obj_position_c, drone_position, drone_orientation=drone_orientation)
                    # print('obj_position_g', obj_position_g)
                    if (obj_position_g[0] - 0)**2+(obj_position_g[1]-0.67)**2 <= 0.05:
                        fitness += 1-result[-2].item()
                    else:
                        fitness += result[-2].item()
            else:
                fitness = 1

            cv2.imshow('Detection result', result_img)
            k = cv2.waitKey(200)
            if k == ord('q') & 0xFF:
                break



    print(fitness)

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def quart_to_rpy(x, y, z, w):
    a = 2 * (w * x + y * z)
    b = 2 * (x * x + y * y)
    c = 2 * (w * y - x * z)
    d = 2 * (w * z + x * y)
    e = 2 * (z * z + y * y)
    if a > 1:
        a = 1
    elif a < -1:
        a = -1
    if b > 1:
        b = 1
    elif b < -1:
        b = -1
    if c > 1:
        c = 1
    elif c < -1:
        c = -1
    if d > 1:
        d = 1
    elif d < -1:
        d = -1
    if e > 1:
        e = 1
    elif e < -1:
        e = -1
    roll = math.atan2(a, 1 - b)
    pitch = math.asin(c)
    yaw = math.atan2(d, 1 - e)
    return roll, pitch, yaw

class AutoLander():
    def __init__(self, border_bits=1):
        """Perform any required setup for the processor during startup
        marker_size: real-world dimensions of markers (i.e. measured after printing)
        marker_spacing: real-world dimensions between (i.e. measured after printing)
        boarder_bits: number of boarder bits used around each marker
        boards: a list of boards to accept during search
        """

        self.name = 'ARUCO'
        self.type = 'fiducial'
        self.encoding = 'mono8'

        # ARUCO Setup
        # Using a sparse pattern (e.g. 4x4) will make it easier to identify,
        # but will make the markers more similar. If >50 are needed, consider
        # using a denser pattern.
        # The 5x5_50 is a good trade-off for uniqueness and size
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()
        self.parameters.markerBorderBits = border_bits

        # General detection parameters
        self.boards = []
        # Generate the boards we will detect
        # for b in boards:
            #ids = numpy.array(b.ids)
        board = aruco.GridBoard_create(2, 2, 2, 0.2, self.dictionary, 0)
            # Manually set our specified IDs
            #board.ids = ids.flatten()
            # Save the board!
        self.boards.append(board)

    @staticmethod
    def get_board_centre_offset(board: cv2.aruco_GridBoard):
        (num_width, num_height) = board.getGridSize()
        total_size_width = board.getMarkerLength() * num_width + board.getMarkerSeparation() * (num_width - 1)
        total_size_height = board.getMarkerLength() * num_height + board.getMarkerSeparation() * (num_height - 1)

        return (total_size_width / 2, total_size_height / 2)


    def process_image(self, image, K, D, debug=True):
        print(111)
        results = []
        image_debug = None

        (corners, ids, _) = aruco.detectMarkers(image=image, dictionary=self.dictionary, parameters=self.parameters)    

        for i in range(len(self.boards)):
            if len(corners) > 0:
                board = self.boards[i]
                # n_found, r_vec, t_vec = aruco.estimatePoseBoard(corners, ids, board, K, D, numpy.zeros((3,1)), numpy.zeros((3,1)))
                r_vec, t_vec, objPoints = aruco.estimatePoseSingleMarkers(corners[0], 1.15, K, D)
                # if r_vec:
                    # First values back are the board coordinates
                    # (xo, yo) = AutoLander.get_board_centre_offset(board)
                    # to = numpy.array([[xo],[yo],[0.0]])
                    # ro = numpy.array([[0.0],[0.0],[0.0]])
                    # (r3, t3, _, _, _, _, _, _, _, _) = cv2.composeRT(ro, to, r_vec, t_vec)
                    # position = Position()
                    # position = [t3[0], t3[1], t3[2]]
                    # position.set_from_vector(t3)
                    # print(position.x, position.y, position.z)
                    # temp = position.x
                    # position.x = -position.y
                    # position.y = -position.y
                    # position.z = 10
                    # print(position.x, position.y, position.z)
                    # rotation = Rotation()
                    # rotation.set_from_r_vec(r3)
                    # rotation = set_from_r_vec(r3)
                    # confidence = float(n_found) / len(board.ids)
                    # print(position)
                    # Each board is ID'd by it's index, and will be unique as long as the defined boards are unique
                    # results.append(Result(id=i, position=position, rotation=rotation, confidence=confidence))
                    # results.append({'id': i, 'position': position, 'rot?ation': rotation, 'confidence': confidence})
                results = {'position': t_vec, 'rotation': r_vec}
                # if debug:
                    # Overlay the board axis
                    #image_debug = aruco.drawAxis(image, K, D, r_vec, t_vec, board.getMarkerLength())
                    # image_debug = cv2.drawFrameAxes(image, K, D, r3, t3, board.getMarkerLength(), 10)
                    # cv2.imshow('Detection result', image_debug)
                    # k = cv2.waitKey(200)
                    # if k == ord('q') & 0xFF:
                    #     break
                    
                    # cv2.imwrite('det.png', image_debug)
        
        print(results)
        return results


def set_from_r_vec(rv:numpy.ndarray):
    if rv.size != 3:
        raise IndexError('Vector must be length 3: rv=%s' % str(rv.size))

    # Pull out angle-axis representation then convert to quaternion
    # Essentially, length is angle and normalized vector is axis
    theta = math.sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2])
    st = math.sin(theta/2)

    w = math.cos(theta/2)
    x = float((rv[0]/theta) * st)
    y = float((rv[1]/theta) * st)
    z = float((rv[2]/theta) * st)

    return [w, x, y, z]

def generate_cam_setting(capture_setting):
    # sensor_width = 23.76
    # sensor_height = 13.365
    sensor_width = 640
    sensor_height = 480
    fov_degrees = capture_setting.fov
    f_x = (sensor_width / 2.0) / math.tan(math.radians(fov_degrees / 2.0))
    f_y = (sensor_height / 2.0) / math.tan(math.radians(fov_degrees / 2.0))
    K = [ f_x, 0.0, sensor_width / 2.0, 0.0, f_y, sensor_height / 2.0, 0.0, 0.0, 1.0 ]
    P = [ f_x, 0.0, sensor_width / 2.0, 0.0, 0.0, f_y, sensor_height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 ]
    D = [0, 0, 0, 0, 0]
    P = [1332.8958740234375, 0.0, 320.0, 0.0, 0.0, 1332.8958740234375, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    K = [1332.8958740234375, 0.0, 320.0, 0.0, 1332.8958740234375, 240.0, 0.0, 0.0, 1.0]
    return {'K': K, 'P': P, 'D': D}

    # sensor_msgs::CameraInfo cam_info_msg;
    # cam_info_msg.header.frame_id = camera_name + "_optical";
    # cam_info_msg.height = capture_setting.height;
    # cam_info_msg.width = capture_setting.width;
    # float f_x = (capture_setting.width / 2.0) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0));
    # // todo focal length in Y direction should be same as X it seems. this can change in future a scene capture component which exactly correponds to a cine camera
    # // float f_y = (capture_setting.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    # cam_info_msg.K = { f_x, 0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 1.0 };
    # cam_info_msg.P = { f_x, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    # return cam_info_msg;


def cb_image(image, camera_info, processor):
    # K = camera_info.proj_mat[]
    K = numpy.array(camera_info['K']).reshape([3, 3])
    D = numpy.array(camera_info['D']).reshape(-1, 1).astype(np.float64)

    response = processor.process_image(image, K, D)
    return response


def coord_convert(obj_position_c, drone_position, drone_orientation):
    _, _, yaw = euler_from_quaternion(drone_orientation.x_val, drone_orientation.y_val,
                                        drone_orientation.z_val, drone_orientation.w_val)
    yaw_init_mat = np.array([[0, 1],[-1, 0]])
    yaw_angle_mat = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    obj_position_delta = np.matmul(obj_position_c, np.matmul(yaw_angle_mat, yaw_init_mat))
    obj_position_g = [obj_position_delta[0] + drone_position.x_val, obj_position_delta[1] + drone_position.y_val]
    return obj_position_g


def pixel_to_pos(z_world, detect_result):
    P = [1332.8958740234375, 0.0, 320.0, 0.0, 0.0, 1332.8958740234375, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    f_x = 1332.8958740234375
    c_x = 320
    f_y = 1332.8958740234375
    c_y = 240

    detect_result = detect_result.detach().cpu().numpy()

    center_x = (detect_result[0] + detect_result[2]) // 2
    center_y = (detect_result[1] + detect_result[3]) // 2

    # print(center_x, center_y)

    x_world = (center_x - c_x) * (-z_world) / f_x
    y_world = (center_y - c_y) * (-z_world) / f_y
    # print(x_world, y_world)
    return [x_world, y_world]


def pos_to_pixel(obj_position, drone_pose):
    position_delta = np.array([obj_position.x_val - drone_pose.position.x_val, obj_position.y_val - drone_pose.position.y_val])
    _, _, yaw = euler_from_quaternion(drone_pose.drone_orientation.x_val, drone_pose.drone_orientation.y_val,
                                        drone_pose.drone_orientation.z_val, drone_pose.drone_orientation.w_val)
    yaw_init_mat = np.array([[0, 1],[-1, 0]])
    yaw_angle_mat = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    rotation_matrix = np.matmul(yaw_angle_mat, yaw_init_mat)
    rotation_inverse = np.linalg.inv(rotation_matrix)
    obj_position_c = np.matmul(position_delta, rotation_inverse)
    print(obj_position_c)



def landing_pipeline(system='yolo', marker_name='marker0', marker_pose=None, weather_params=None):
    # initialize the landing scenario
    if not marker_pose:
        marker_pose = set_landing_scenario(marker_name=marker_name, weather_params=weather_params)
    else:
        drone_pose = client.simGetVehiclePose('Copter')
        take_off(drone_pose, 15)



    # get current position of the drone
    drone_position = client.simGetVehiclePose('Copter').position

    # let the drone fly to the top of the marker
    client.moveToPositionAsync(marker_pose.position.x_val, marker_pose.position.y_val, drone_position.z_val, 3)

    # load landing system
    if system == 'yolo':
        model = load_model()
        model = model.cuda()
        opt = parse_opt()

    detect_marker_position = None
    fly_to = False
    # during the flight, once detecting the marker, try to land
    while True:

        # drone_state = client.getMultirotorState('Copter')
        # print('drone state: ', drone_state)
        # get detection result
        image = get_current_scene(0)
        drone_position = client.simGetVehiclePose('Copter').position
        drone_orientation = client.simGetVehiclePose('Copter').orientation
        # if fly_to and drone_position.z_val >= -1:
        #     time.sleep(3)
        #     break

        detect_result, result_img = detect(model, image, opt)
        print(detect_result)
        detect_result_post = None
        detect_result_conf = 0
        for result in detect_result[0]:
            if result[-2] > 0.8 and result[-2] > detect_result_conf:
                detect_result_post = result
                detect_result_conf = result[-2]

        if detect_result_post != None:
            obj_position_c = pixel_to_pos(drone_position.z_val, detect_result_post)
            print('obj_position_c: ', obj_position_c)



            obj_position_g = coord_convert(obj_position_c, drone_position, drone_orientation=drone_orientation)
            print('obj_position_g', obj_position_g)

            # if detect_marker_position == None:
            #     detect_marker_position = obj_position_g
            # else:
            #     if 
            # if not fly_to:
            client.moveToPositionAsync(obj_position_g[0], obj_position_g[1], 1.5, 3)
            fly_to = True
            # while True:
            #     image = get_current_scene(0)
            #     detect_result, result_img = detect(model, image, opt)
            cv2.imshow('Detection result', result_img)
            k = cv2.waitKey(200)
            # if k == ord('q') & 0xFF:
            #     break
                

            # if drone_state.landed_state != 0:
            #     break
        else:
            print("No marker detected")        

def pipeline_yolo(marker_name='Maker_single'):
    reset(marker_name)
    # weather = np.random.random(11)
    # set_current_weather(weather)
    # marker_pose = respawn_marker(marker_name='Marker_single')
    # init_drone()
    # take_off()
    # client.moveToPositionAsync(obj_position_g[0], obj_position_g[1], 1.5, 3).join()


    # init()
    model = load_model()
    model = model.cuda()
    opt = parse_opt()
    time.sleep(3)


    drone_position = client.simGetVehiclePose('Copter').position
    drone_pose = client.simGetVehiclePose('Copter').orientation
    # get detection result
    image = get_current_scene(0)
    detect_result, result_img = detect(model, image, opt)
    print(detect_result)
    detect_result_post = None
    detect_result_conf = 0
    for result in detect_result[0]:
        if result[-2] > 0.8 and result[-2] > detect_result_conf:
            detect_result_post = result
            detect_result_conf = result[-2]

    if detect_result_post != None:
        obj_position_c = pixel_to_pos(drone_position.z_val, detect_result_post)
        print('obj_position_c: ', obj_position_c)
        obj_position_g = coord_convert(obj_position_c, drone_position, drone_orientation=drone_pose)
        print('obj_position_g', obj_position_g)

        client.moveToPositionAsync(obj_position_g[0], obj_position_g[1], 1.5, 3)
        while True:
            image = get_current_scene(0)
            detect_result, result_img = detect(model, image, opt)
            cv2.imshow('Detection result', result_img)
            k = cv2.waitKey(200)
            if k == ord('q') & 0xFF:
                break
        

    else:
        print("No marker detected")

def pipeline_opencv(marker_type='single'):
    init()


def yolo_test():
    # init()
    model = load_model()
    model = model.cuda()
    opt = parse_opt()
    time.sleep(3)

    drone_pose = client.simGetVehiclePose('Copter')
    take_off(drone_pose)

    weather_count = 0

    while True:
        if weather_count == 50:
            params = np.random.random(11)
            set_current_weather(params)
            weather_count = 0
            # time.sleep(3)
            x = random.randint(-3, 3)
            y = random.randint(-5, 5)
            z = random.randint(10, 30)
            print(x, y, z)
            fly_to(x, y, -z)
    
        weather_count += 1
        # drone_state = client.getMultirotorState('Copter')
        # print('drone state: ', drone_state)
        # get detection result
        image = get_current_scene(0)
        detect_result, result_img = detect(model, image, opt)
        # print(detect_result)
        
        cv2.imshow('Detection result', result_img)
        k = cv2.waitKey(200)

def yolo_test_2():
    # 'marker_blur', 'marker_brightness_1', 'marker_brightness_2', 'marker_contrast',
    # markers = ['marker0', 'marker_shear',
    #             'marker_translation', 'non_marker_1', 'non_marker_2', 'non_marker_3']
    markers = ['marker0', 'non_marker_1']
    # markers = ['non_marker_3']
    for marker in markers:
        pipeline_yolo(marker)
        # weather = np.random.random(11)

        landing_pipeline(marker_name=marker)


def param_test(params):
    model = load_model()
    model = model.cuda()
    opt = parse_opt()
    time.sleep(3)
    fitness_func(model, opt, params)


def dynamic_landing_test():
    marker_pose = client.simGetObjectPose('npc_vehicle_2')
    marker_pose.position.y_val = -50
    marker_pose.position.z_val = -15
    print('marker_pose: ', marker_pose)
    landing_pipeline(marker_pose=marker_pose)


def calculate_output_length(length_in, kernel_size, stride=1, padding=0, dilation=1):
    return (length_in + 2 * padding - dilation * (kernel_size - 1) - 1) // stride + 1

if __name__ == "__main__":
    # pipeline_yolo()
    param_test([2.0805,       1.166,      2.1669,     0.39622,     0.70427,     0.27522,      1.2032])
    # dynamic_landing_test()
    # print(calculate_output_length(128*128, 11, 1, 5))

 


