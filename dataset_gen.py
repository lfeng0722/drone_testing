from airsim_api import *
import cv2


def get_rectangle(points):
    # for p in points[0]:
    #     print(p)
    min_x = min([point[0] for point in points[0]])
    max_x = max([point[0] for point in points[0]])
    min_y = min([point[1] for point in points[0]])
    max_y = max([point[1] for point in points[0]])

    return min_x, max_x, min_y, max_y

def marker_detedction(image):
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
                                                parameters=arucoParams)

    return corners, ids


def no_marker_data_gen(num_image=2000):
    params = [0] * 11
    set_current_weather(params)
    for i in range(500, num_image):
        add_noise = random.randint(0, 15)
        if add_noise == 1:
            marker_pose = respawn_marker('non_marker_1')
            drone_pose = respawn_drone(marker_pose=marker_pose)
            take_off(drone_pose)

        elif add_noise == 2:
            marker_pose = respawn_marker('non_marker_2')
            drone_pose = respawn_drone(marker_pose=marker_pose)
            take_off(drone_pose)

        elif add_noise == 3:
            marker_pose = respawn_marker('non_marker_3')
            drone_pose = respawn_drone(marker_pose=marker_pose)
            take_off(drone_pose)
        
        else:
            reset(marker_name=None)

        add_weather = random.randint(0, 5)
        if add_weather == 1:
            params = [random.random() for _ in range(11)]
            set_current_weather(params)
        
        time.sleep(1)
        current_scene = get_current_scene()
        cv2.imwrite('datasets/images_no_marker/scene{}.png'.format(i), current_scene)


def bad_weather_data_gen():
    label_txt = open('datasets/label_class_bad_weather.txt', 'a')
    save_data = True
    for marker_id in range(15):
        collected = 0
        scene = 0
        while collected < 40:
            # reset the scene
            current_scene = reset(marker_id)
            time.sleep(4)
            # try to detect the marker
            try:
                corners, ids = marker_detedction(current_scene)
            except:
                continue

            print(corners)

            if len(corners) == 0:
                continue

            x1, x2, y1, y2 = get_rectangle(corners[0])
            labeled_image = cv2.rectangle(current_scene, (int(x1), int(y1)), (int(x2), int(y2)), (255,0,0), 2)

            if save_data:
                params = [random.random() for _ in range(11)]
                set_current_weather(params)
                time.sleep(random.randint(0, 5))
                scene_img = get_current_scene()
                cv2.imwrite('datasets/images_class/class{}_scene{}_bad_weather.png'.format(marker_id, scene), scene_img)
                labeled_image = cv2.rectangle(scene_img, (int(x1), int(y1)), (int(x2), int(y2)), (255,0,0), 2)
                cv2.imwrite('datasets/labels_class/class{}_scene{}_bad_weather.png'.format(marker_id, scene), labeled_image)  
                label_txt.write("{} {} {} {} {}\n".format(int(x1), int(x2), int(y1), int(y2), marker_id))
                label_txt.flush()
                time.sleep(1)

            collected += 1    
            scene += 1

def normal_data_gen():
    label_txt = open('datasets/label_0.txt', 'a')
    save_data = True

    for marker_id in range(1):
        collected = 0
        scene = 0
        while collected < 2000:
            # reset the scene
            current_scene, marker_pose = reset('marker0')
            time.sleep(3)
            # try to detect the marker
            try:
                corners, ids = marker_detedction(current_scene)
            except:
                continue

            print(corners)

            # rect = cv2.minAreaRect(corners[0])
            if len(corners) == 0:
                continue
            else:

                add_noise = random.randint(0, 15)
                if add_noise == 1:
                    respawn_marker('non_marker_1', marker_pose)
                elif add_noise == 2:
                    respawn_marker('non_marker_2', marker_pose)
                elif add_noise == 3:
                    respawn_marker('non_marker_3', marker_pose)


                scene_img = current_scene.copy()
                x1, x2, y1, y2 = get_rectangle(corners[0])
                image_debug = cv2.aruco.drawDetectedMarkers(current_scene, corners, ids)
                labeled_image = cv2.rectangle(current_scene, (int(x1), int(y1)), (int(x2), int(y2)), (255,0,0), 2)
                cv2.imshow('Detection result', image_debug)
                k = cv2.waitKey(200)
                if k == ord('q') & 0xFF:
                    break

            #     time.sleep(3)

            x1, x2, y1, y2 = get_rectangle(corners[0])

            
            if save_data:
                label_txt.write("{} {} {} {} {}\n".format(int(x1), int(x2), int(y1), int(y2), marker_id))
                label_txt.flush()
                cv2.imwrite('datasets/images_0/class{}_scene{}.png'.format(marker_id, scene), scene_img)
                cv2.imwrite('datasets/labels_0/class{}_label{}.png'.format(marker_id, scene), labeled_image)
            
                # label_txt.write("{} {} {} {} {}\n".format(int(x1), int(x2), int(y1), int(y2), marker_id))
                # label_txt.flush()
                time.sleep(1)

            collected += 1    
            scene += 1


normal_data_gen()
no_marker_data_gen()