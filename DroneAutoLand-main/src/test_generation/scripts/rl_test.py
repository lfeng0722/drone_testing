#!/usr/bin/env python3
import rospy
import time
from test_generation.rl.networks import DQNAgent
from airsim_simulation.sim_env import AirSimEnv
from marker_detection.detectors.tphyolo_detector import TphyoloDectector
from airsim_simulation.scenario import sample_scenario, sample_test_scenario
from airsim_simulation.components import Pose
import cv2
import numpy as np
import random
from torch.utils.tensorboard import SummaryWriter
import math
# random.seed(0)
SEMANTIC_PIXEL_MAX = 1056
SEMANTIC_VALUE = 47


def calculate_3d_distance(point1, point2):
    """Calculate the Euclidean distance between two points in 3D space."""
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5

def random_point_in_circle(radius):
    # Random angle
    angle = random.uniform(0, 2 * math.pi)
    # Random radius
    r = radius * math.sqrt(random.uniform(0, 1))
    # Calculating coordinates
    x = r * math.cos(angle)
    y = r * math.sin(angle)
    return x, y


if __name__ == '__main__':
    rospy.init_node('RL_test', anonymous=True)
    env = AirSimEnv('', '127.0.0.1')
    DQN = DQNAgent(15, [m for m in range(23)])
    mode = 'BA_Training'
    # scenario = sample_test_scenario()
    memory = []
    radius = 5
    batch_size = 128
    # print(scenario)
    # scenario = env.validate_scenario(scenario)
    time.sleep(1)
    # env.set_scenario(scenario)
    detector = TphyoloDectector('/media/linfeng/HDD1/autoland/DroneAutoLand/src/marker_detection/src/marker_detection/tphyolo/runs/best_new.pt')
    train_writer = SummaryWriter(log_dir='/media/linfeng/HDD1/autoland/DroneAutoLand-main/src/test_generation/src/test_generation/rl/loss')
    train_writer1 = SummaryWriter(log_dir='/media/linfeng/HDD1/autoland/DroneAutoLand-main/src/test_generation/src/test_generation/rl/reward')
    # for epoch in range(1000):
    epoch = 0
    if mode == 'train':

        while rospy.is_shutdown() is False:

            scenario = sample_test_scenario()

            UAV_respawn_point = random_point_in_circle(radius)
            env.surrograte_trianing(scenario.tp_marker, UAV_respawn_point)
            env.set_scenario(scenario)
            env.load_scenario()

            # time.sleep(1)

            env.set_segmentation()

            # print(env.dynamic_objects)

            # time.sleep(1)

            # drone_start_position = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y - 15)


            marker_poseition = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y,scenario.tp_marker.pose.z)
            # print(marker_poseition)


            done = False
            acc_reward = 0
            count_time =0
            while not done:
                count_time+=1
                actor = env.dynamic_objects[0]
                # for i, actor in enumerate(env.dynamic_objects):
                actor_pose = env.get_pose(actor)
                actor_position = (actor_pose.position.x_val, actor_pose.position.y_val, actor_pose.position.z_val)


                UAV_current_pose = env.get_uav_pose()
                UAV_current_position = (UAV_current_pose.position.x_val, UAV_current_pose.position.y_val, UAV_current_pose.position.z_val)

                UAV_obj_x = actor_pose.position.x_val-UAV_current_pose.position.x_val
                UAV_obj_y = actor_pose.position.y_val-UAV_current_pose.position.y_val
                UAV_obj_z = actor_pose.position.z_val - UAV_current_pose.position.z_val

                UAV_marker_x =  UAV_current_pose.position.x_val -marker_poseition[0]
                UAV_marker_y = UAV_current_pose.position.y_val - marker_poseition[1]
                UAV_marker_z = UAV_current_pose.position.z_val - marker_poseition[2]

                obj_marker_x = actor_pose.position.x_val - marker_poseition[0]
                obj_marker_y = actor_pose.position.y_val- marker_poseition[1]
                obj_marker_z = actor_pose.position.z_val- marker_poseition[2]

                state  = (UAV_marker_x, UAV_marker_y,  obj_marker_x,obj_marker_y)

                # action = DQN.get_action(state)
                action = DQN.get_action(state)
                # print(action)

                # action  = random.randint(0, 4)
                print(actor, action)
                if action == 1:
                    env.set_npc_forward(actor, 1)
                elif action == 2:
                    env.set_npc_backward(actor, 1)
                elif action == 3:
                    env.set_npc_left(actor, 1)
                elif action == 4:
                    env.set_npc_right(actor, 1)
                time.sleep(0.1)
                # scene_img, _ = env.get_current_scene(image_encoding='bgr')
                # semantic_img, _ = env.get_current_scene(image_type=5)
                # cv2.imshow('1111',semantic_img)
                # cv2.waitKey(0)
                # semantic_img = cv2.cvtColor(semantic_img, cv2.COLOR_BGR2GRAY)
                # #
                # unique_values, counts = np.unique(semantic_img, return_counts=True)
                # print('1111111',counts)
                # value_counts = dict(zip(unique_values, counts))
                # # print(value_counts)
                # ratio = value_counts[SEMANTIC_VALUE] / SEMANTIC_PIXEL_MAX
                # if ratio > 0.96:
                #     ratio = 1
                # cv2.imshow('scene_img', semantic_img)
                # cv2.waitKey()
                # reward = ((1/ratio)-1)*10
                reward = 0

                actor_pose = env.get_pose(actor)
                actor_position = (actor_pose.position.x_val, actor_pose.position.y_val, actor_pose.position.z_val)

                UAV_current_pose = env.get_uav_pose()
                UAV_current_position = (
                UAV_current_pose.position.x_val, UAV_current_pose.position.y_val, UAV_current_pose.position.z_val)

                UAV_obj_x = actor_pose.position.x_val - UAV_current_pose.position.x_val
                UAV_obj_y = actor_pose.position.y_val - UAV_current_pose.position.y_val
                UAV_obj_z = actor_pose.position.z_val - UAV_current_pose.position.z_val

                UAV_marker_x = UAV_current_pose.position.x_val - marker_poseition[0]
                UAV_marker_y = UAV_current_pose.position.y_val - marker_poseition[1]
                UAV_marker_z = UAV_current_pose.position.z_val - marker_poseition[2]

                obj_marker_x = actor_pose.position.x_val - marker_poseition[0]
                obj_marker_y = actor_pose.position.y_val - marker_poseition[1]
                obj_marker_z = actor_pose.position.z_val - marker_poseition[2]

                next_state = (
                UAV_marker_x, UAV_marker_y, obj_marker_x, obj_marker_y)

                UAV_marker_dis = calculate_3d_distance(UAV_current_position, marker_poseition)

                # print(UAV_current_position)
                if UAV_marker_dis<0.5:
                    done = True
                    print('landed')


                if env.get_collision_info() == actor:
                    reward+=1
                    done = True
                    print(env.get_collision_info())
                    print('collided')


                obj_marker_distance = calculate_3d_distance(actor_position, marker_poseition)
                if obj_marker_distance<1:
                    reward+= 0.1/obj_marker_distance

                acc_reward += reward

                memory.append((state, action, reward, next_state, done))

                if len(memory) > batch_size:
                        batch = random.sample(memory, batch_size)
                        loss = DQN.train([(s, a, r, ns, d) for s, a, r, ns, d in batch])
                if count_time>500:
                    break
            train_writer1.add_scalar(
                tag="/media/linfeng/HDD1/autoland/DroneAutoLand-main/src/test_generation/src/test_generation/rl/reward",
                scalar_value=acc_reward,
                global_step=epoch)
            if len(memory) > batch_size:
                train_writer.add_scalar(
                    tag="/media/linfeng/HDD1/autoland/DroneAutoLand-main/src/test_generation/src/test_generation/rl/loss",
                    scalar_value=loss,
                    global_step=epoch)
            epoch += 1
            if epoch % 30 ==0 :
                DQN.update_target()
            time.sleep(5)
        # time.sleep(3)
        #     drone_current_position = (drone_start_position[0], drone_start_position[1] + i + 1)
        #     actor_pose = env.get_pose(env.dynamic_objects[0])
        #     actor_position = (actor_pose.position.x_val, actor_pose.position.y_val)
        #     next_state = (actor_position[0]-marker_poseition[0],actor_position[1]-marker_poseition[1],drone_current_position[0]-marker_poseition[0],drone_current_position[1]-marker_poseition[1])



        #     scene_img, _ = env.get_current_scene(image_encoding='bgr')
        #     semantic_img, _ = env.get_current_scene(image_type=5)
        #     semantic_img = cv2.cvtColor(semantic_img, cv2.COLOR_BGR2GRAY)


        #     # Get unique pixel values and their counts
        #     unique_values, counts = np.unique(semantic_img, return_counts=True)
        #     value_counts = dict(zip(unique_values, counts))
        #     # print(value_counts)
        #     ratio = value_counts[SEMANTIC_VALUE] / SEMANTIC_PIXEL_MAX
        #     if ratio > 0.96:
        #         ratio = 1
        #     # cv2.imshow('scene_img', semantic_img)
        #     # cv2.waitKey()
        #     reward = ((1/ratio)-1)*10
        #     # print(ratio)
        #     acc_reward+=reward
        #     memory.append((state, action, reward, next_state))
            
            
        #     # detection_result, debug_image = detector.detect(scene_img)
        #     # if detection_result:
        #     #     confidence = detection_result.confidence
        #     # else:
        #     #     confidence = 0

        #     # print(ratio, confidence)




        #     # time.sleep(1)
    elif mode =='test':
        while rospy.is_shutdown() is False:
            scenario = sample_test_scenario()

            UAV_respawn_point = random_point_in_circle(radius)
            env.surrograte_trianing(scenario.tp_marker, UAV_respawn_point)
            env.set_scenario(scenario)
            env.load_scenario()

            env.set_segmentation()
            marker_poseition = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y, scenario.tp_marker.pose.z)

            done = False

            while not done:

                # actor = env.dynamic_objects[0]
                for i, actor in enumerate(env.dynamic_objects):
                    actor_pose = env.get_pose(actor)
                    actor_position = (actor_pose.position.x_val, actor_pose.position.y_val, actor_pose.position.z_val)

                    UAV_current_pose = env.get_uav_pose()
                    UAV_current_position = (
                    UAV_current_pose.position.x_val, UAV_current_pose.position.y_val, UAV_current_pose.position.z_val)

                    UAV_obj_x = actor_pose.position.x_val - UAV_current_pose.position.x_val
                    UAV_obj_y = actor_pose.position.y_val - UAV_current_pose.position.y_val
                    UAV_obj_z = actor_pose.position.z_val - UAV_current_pose.position.z_val

                    UAV_marker_x = UAV_current_pose.position.x_val - marker_poseition[0]
                    UAV_marker_y = UAV_current_pose.position.y_val - marker_poseition[1]
                    UAV_marker_z = UAV_current_pose.position.z_val - marker_poseition[2]

                    obj_marker_x = actor_pose.position.x_val - marker_poseition[0]
                    obj_marker_y = actor_pose.position.y_val - marker_poseition[1]
                    obj_marker_z = actor_pose.position.z_val - marker_poseition[2]

                    state = (UAV_marker_x, UAV_marker_y, obj_marker_x, obj_marker_y)

                    # action = DQN.get_action(state)
                    action = DQN.evaluate(state)
                    # print(action)

                    # action  = random.randint(0, 4)
                    print(actor, action)
                    if action == 1:
                        env.move_npc_forward(actor, 1)
                    elif action == 2:
                        env.move_npc_backward(actor, 1)
                    elif action == 3:
                        env.move_npc_left(actor, 1)
                    elif action == 4:
                        env.move_npc_right(actor, 1)
                    time.sleep(0.1)

                    actor_pose = env.get_pose(actor)
                    actor_position = (actor_pose.position.x_val, actor_pose.position.y_val, actor_pose.position.z_val)

                    UAV_current_pose = env.get_uav_pose()
                    UAV_current_position = (
                        UAV_current_pose.position.x_val, UAV_current_pose.position.y_val, UAV_current_pose.position.z_val)

                    UAV_obj_x = actor_pose.position.x_val - UAV_current_pose.position.x_val
                    UAV_obj_y = actor_pose.position.y_val - UAV_current_pose.position.y_val
                    UAV_obj_z = actor_pose.position.z_val - UAV_current_pose.position.z_val

                    UAV_marker_x = UAV_current_pose.position.x_val - marker_poseition[0]
                    UAV_marker_y = UAV_current_pose.position.y_val - marker_poseition[1]
                    UAV_marker_z = UAV_current_pose.position.z_val - marker_poseition[2]

                    obj_marker_x = actor_pose.position.x_val - marker_poseition[0]
                    obj_marker_y = actor_pose.position.y_val - marker_poseition[1]
                    obj_marker_z = actor_pose.position.z_val - marker_poseition[2]


                    UAV_marker_dis = calculate_3d_distance(UAV_current_position, marker_poseition)


                    if UAV_marker_dis < 0.5:
                        done = True
                        print('landed')

                    if env.get_collision_info() == actor:
                        done = True
                        print(env.get_collision_info())
                        print('collided')
    elif mode == 'BA_Training':
        while rospy.is_shutdown() is False:

            scenario = sample_test_scenario()

            weather = scenario.weather.to_vec()
            daytime = scenario.time.to_vec()

            UAV_respawn_point = random_point_in_circle(radius)
            env.surrograte_trianing(scenario.tp_marker, UAV_respawn_point)
            env.set_scenario(scenario)
            env.load_scenario()
            # time.sleep(1)
            env.set_segmentation()
            # print(env.dynamic_objects)

            # time.sleep(1)

            # drone_start_position = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y - 15)

            marker_poseition = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y,scenario.tp_marker.pose.z)
            # print(marker_poseition)

            done = False
            acc_reward = 0
            count_time =0
            while not done:
                count_time+=1
                actor = env.dynamic_objects[0]
                # for i, actor in enumerate(env.dynamic_objects):
                actor_pose = env.get_pose(actor)
                actor_position = (actor_pose.position.x_val, actor_pose.position.y_val, actor_pose.position.z_val)


                UAV_current_pose = env.get_uav_pose()
                UAV_current_position = (UAV_current_pose.position.x_val, UAV_current_pose.position.y_val, UAV_current_pose.position.z_val)

                UAV_obj_x = actor_pose.position.x_val-UAV_current_pose.position.x_val
                UAV_obj_y = actor_pose.position.y_val-UAV_current_pose.position.y_val
                UAV_obj_z = actor_pose.position.z_val - UAV_current_pose.position.z_val

                UAV_marker_x =  UAV_current_pose.position.x_val -marker_poseition[0]
                UAV_marker_y = UAV_current_pose.position.y_val - marker_poseition[1]
                UAV_marker_z = UAV_current_pose.position.z_val - marker_poseition[2]

                obj_marker_x = actor_pose.position.x_val - marker_poseition[0]
                obj_marker_y = actor_pose.position.y_val- marker_poseition[1]
                obj_marker_z = actor_pose.position.z_val- marker_poseition[2]

                pos = (UAV_marker_x, UAV_marker_y,  obj_marker_x, obj_marker_y)
                state= weather + daytime +list(pos)


                # action = DQN.get_action(state)
                action = DQN.get_action(state)
                # print(action)

                # action  = random.randint(0, 4)
                # print(actor, action)
                if action == 1:
                    env.set_npc_forward(actor, 1)
                if action == 2:
                    env.set_npc_backward(actor, 1)
                if action == 3:
                    env.set_npc_left(actor, 1)
                if action == 4:
                    env.set_npc_right(actor, 1)
                if action == 5:
                    weather[0] += 0.01
                    print('increase rain')
                if action == 6:
                    weather[0] -= 0.01
                    print('decrease rain')
                if action == 7:
                    weather[1] += 0.01
                    print('increase Roadwetness')
                if action == 8:
                    weather[1] -= 0.01
                    print('decrease Roadwetness')
                if action == 9:
                    weather[2] += 0.01
                    print('increase Snow')
                if action == 10:
                    weather[2] -= 0.01
                    print('decrease Snow')
                if action == 11:
                    weather[3] += 0.01
                    print('increase RoadSnow')
                if action == 12:
                    weather[3] -= 0.01
                    print('decrease RoadSnow')
                if action == 13:
                    weather[4] += 0.01
                    print('increase MapleLeaf')
                if action == 14:
                    weather[4] -= 0.01
                    print('decrease MapleLeaf')
                if action == 15:
                    weather[5] += 0.01
                    print('increase RoadLeaf')
                if action == 16:
                    weather[5] -= 0.01
                    print('decrease RoadLeaf')
                if action == 17:
                    weather[6] += 0.01
                    print('increase Dust')
                if action == 18:
                    weather[6] -= 0.01
                    print('decrease Dust')
                if action == 19:
                    weather[7] += 0.01
                    print('increase Fog')
                if action == 20:
                    weather[7] -= 0.01
                    print('decrease Fog')
                if action == 21:
                    daytime[0] += 0.01
                    print('increase time')
                if action == 22:
                    daytime[1] -= 0.01
                    print('decrease time')
                time.sleep(0.1)

                env.set_weather(weather)
                env.set_time_of_day(daytime)
                # scene_img, _ = env.get_current_scene(image_encoding='bgr')
                # semantic_img, _ = env.get_current_scene(image_type=5)
                # cv2.imshow('1111',semantic_img)
                # cv2.waitKey(0)
                # semantic_img = cv2.cvtColor(semantic_img, cv2.COLOR_BGR2GRAY)
                # #
                # unique_values, counts = np.unique(semantic_img, return_counts=True)
                # print('1111111',counts)
                # value_counts = dict(zip(unique_values, counts))
                # # print(value_counts)
                # ratio = value_counts[SEMANTIC_VALUE] / SEMANTIC_PIXEL_MAX
                # if ratio > 0.96:
                #     ratio = 1
                # cv2.imshow('scene_img', semantic_img)
                # cv2.waitKey()
                # reward = ((1/ratio)-1)*10
                reward = 0

                actor_pose = env.get_pose(actor)
                actor_position = (actor_pose.position.x_val, actor_pose.position.y_val, actor_pose.position.z_val)

                UAV_current_pose = env.get_uav_pose()
                UAV_current_position = (
                UAV_current_pose.position.x_val, UAV_current_pose.position.y_val, UAV_current_pose.position.z_val)

                UAV_obj_x = actor_pose.position.x_val - UAV_current_pose.position.x_val
                UAV_obj_y = actor_pose.position.y_val - UAV_current_pose.position.y_val
                UAV_obj_z = actor_pose.position.z_val - UAV_current_pose.position.z_val

                UAV_marker_x = UAV_current_pose.position.x_val - marker_poseition[0]
                UAV_marker_y = UAV_current_pose.position.y_val - marker_poseition[1]
                UAV_marker_z = UAV_current_pose.position.z_val - marker_poseition[2]

                obj_marker_x = actor_pose.position.x_val - marker_poseition[0]
                obj_marker_y = actor_pose.position.y_val - marker_poseition[1]
                obj_marker_z = actor_pose.position.z_val - marker_poseition[2]

                pos = (
                UAV_marker_x, UAV_marker_y, obj_marker_x, obj_marker_y)
                next_state = weather+daytime+list(pos)


                UAV_marker_dis = calculate_3d_distance(UAV_current_position, marker_poseition)

                # print(UAV_current_position)
                if UAV_marker_dis<0.5:
                    done = True
                    print('landed')


                if env.get_collision_info() == actor:
                    reward+=1
                    done = True
                    print(env.get_collision_info())
                    print('collided')


                obj_marker_distance = calculate_3d_distance(actor_position, marker_poseition)
                if obj_marker_distance<1:
                    reward+= 0.1/obj_marker_distance

                acc_reward += reward

                memory.append((state, action, reward, next_state, done))

                if len(memory) > batch_size:
                        batch = random.sample(memory, batch_size)
                        loss = DQN.train([(s, a, r, ns, d) for s, a, r, ns, d in batch])
                if count_time>500:
                    break
            train_writer1.add_scalar(
                tag="/media/linfeng/HDD1/autoland/DroneAutoLand-main/src/test_generation/src/test_generation/rl/reward",
                scalar_value=acc_reward,
                global_step=epoch)
            if len(memory) > batch_size:
                train_writer.add_scalar(
                    tag="/media/linfeng/HDD1/autoland/DroneAutoLand-main/src/test_generation/src/test_generation/rl/loss",
                    scalar_value=loss,
                    global_step=epoch)
            epoch += 1
            if epoch % 30 ==0 :
                DQN.update_target()
            time.sleep(5)
        
    # rospy.spin()

    # train_writer.close()