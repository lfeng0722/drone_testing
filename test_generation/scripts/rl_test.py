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

# random.seed(0)
SEMANTIC_PIXEL_MAX = 1056
SEMANTIC_VALUE = 47

if __name__ == '__main__':
    rospy.init_node('RL_test', anonymous=True)
    env = AirSimEnv('cv', '10.6.37.180')
    DQN = DQNAgent(4,[0, 1, 2, 3, 4]) 
    DQN_2 = DQNAgent(4,[0, 1, 2, 3, 4]) 
    DQN_arr = [DQN, DQN_2]
    scenario = sample_test_scenario()
    memory = []
    batch_size = 64
    print(scenario)
    # scenario = env.validate_scenario(scenario)
    time.sleep(1)
    # env.set_scenario(scenario)
    detector = TphyoloDectector('/home/yao/Documents/DroneAutoLand/src/marker_detection/src/marker_detection/tphyolo/runs/best_new.pt')
    # train_writer = SummaryWriter(log_dir='/home/yao/Documents/DroneAutoLand/src/test_generation/src/rl/loss')
    # train_writer1 = SummaryWriter(log_dir='/home/yao/Documents/DroneAutoLand/src/test_generation/src/rl/reward')
    # for epoch in range(1000):
    epoch = 0
    while rospy.is_shutdown() is False:

        scenario = sample_test_scenario()
        env.set_scenario(scenario)
        env.load_scenario()
        time.sleep(1)
        env.set_segmentation()
        print(env.dynamic_objects)
        time.sleep(1)
        drone_start_position = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y - 15)
        acc_reward = 0
     
        for i in range(20):
            drone_current_position = (drone_start_position[0], drone_start_position[1] + i)
            for i, actor in enumerate(env.dynamic_objects):

                marker_poseition = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y)
                actor_pose = env.get_pose(env.dynamic_objects[i])
                actor_position = (actor_pose.position.x_val, actor_pose.position.y_val)
                state  = (actor_position[0]-marker_poseition[0],actor_position[1]-marker_poseition[1],drone_current_position[0]-marker_poseition[0],drone_current_position[1]-marker_poseition[1])
                # action = DQN.get_action(state)
                action = DQN_arr[i].evaluate(state)
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
            

            pose = Pose(drone_current_position[0], drone_current_position[1], 10, 0)
            env.set_drone_pose(pose) 
            time.sleep(0.3)
            
        epoch += 1
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
        # train_writer1.add_scalar(tag="/home/yao/Documents/DroneAutoLand/src/test_generation/src/rl/reward", scalar_value=acc_reward,
        #                             global_step=epoch)
        # if len(memory) > batch_size:
        #         batch = random.sample(memory, batch_size)

        #         loss = DQN.train([(s, a, r, ns) for s, a, r, ns in batch])

        #         train_writer.add_scalar(tag="/home/yao/Documents/DroneAutoLand/src/test_generation/src/rl/loss", scalar_value=loss,
        #                             global_step=epoch)
                
        # if epoch % 50 ==0 :
        #     DQN.update_target()
        
    # rospy.spin()

    # train_writer.close()