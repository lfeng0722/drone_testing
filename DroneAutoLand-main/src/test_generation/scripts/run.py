#!/usr/bin/env python3
from test_generation.ga.online_test2 import online
from test_generation.ga.offline_fuzzer import Offline
from test_generation.ga.RL_online_testing import RL_online
from test_generation.ga.random_baseline import Random
from test_generation.ga.rlaga import RLAGA
from test_generation.rl.networks import DQNAgent
import rospy
from airsim_simulation.sim_env import AirSimEnv
from airsim_simulation.scenario import Scenario as ScenarioObj



if __name__ == '__main__':
    rospy.init_node('rlaga_search', anonymous=True)
    # rospy.Rate(1)
    rospy.loginfo("Starting rlaga_search processor...")

    env = '127.0.0.1'
    pop_size = 10
    generations = 21
    mutation_rate = 0.2
    rl_agent = DQNAgent
    scenario_topic = '/simulation/scenario'
    map_name = 'court'
    result = ''
    rlaga = RLAGA(rl_agent, pop_size, generations, mutation_rate, env, scenario_topic,  map_name, result_save_path=result)

    # scenario_dict = {'tp_marker': {'id': 0, 'material': 'normal', 'pos_x': 6.138451334680438, 'pos_y': -26.12248323669282, 'pos_z': 0.0, 'angle': 142.5606399586289}, 'drone_start_pose': {'pos_x': 23.16930902743197, 'pos_y': -3.24027642333159, 'pos_z': 0.5, 'angle': 312.1922115592206}, 'gps_pose': {'pos_x': 7.8387773380079295, 'pos_y': -21.41351939275021, 'pos_z': 0.0, 'angle': 81.65824278537802}, 'radius': 7.5, 'weather': {'rain': 0, 'road_wetness': 0.15539694293247108, 'snow': 0, 'road_snow': 0.0, 'maple_leaf': 0, 'road_leaf': 0.0, 'dust': 0, 'fog': 0.0, 'wind': 0.23315301117483667}, 'time': {'hour': 0.8316703068531409, 'minute': 0.25530748488727606}, 'fp_markers': [{'id': 1, 'material': 'normal', 'pos_x': 10.432014849156573, 'pos_y': -16.318627337519857, 'pos_z': 0.0, 'angle': 108.40622633340696}, {'id': 2, 'material': 'normal', 'pos_x': 5.686654609983478, 'pos_y': -13.045964868012181, 'pos_z': 0.0, 'angle': 55.49267345759791}], 'actors': [{'type': 'person_2', 'start_pose': {'pos_x': 9.462404898826765, 'pos_y': -26.42530382829085, 'pos_z': 0.5, 'angle': 190.9693431002773}, 'end_pose': {'pos_x': 4.424440260920163, 'pos_y': -24.72366803652573, 'pos_z': 0.5, 'angle': 78.34638962961739}, 'speed': 0.05165899663307494}, {'type': 'person_2', 'start_pose': {'pos_x': 2.516197063354194, 'pos_y': -37.14606072036269, 'pos_z': 0.5, 'angle': 210.33873202697703}, 'end_pose': {'pos_x': 5.08900911422247, 'pos_y': -20.972462065807942, 'pos_z': 0.5, 'angle': 240.6215411690469}, 'speed': 0.34303529326973925}, {'type': 'person_2', 'start_pose': {'pos_x': -1.6144258296464185, 'pos_y': -18.598885089449862, 'pos_z': 0.5, 'angle': 162.4505926501285}, 'end_pose': {'pos_x': -5.17803507800962, 'pos_y': -23.449677474327753, 'pos_z': 0.5, 'angle': 200.8004686487389}, 'speed': 0.20879692398381178}]}

    # scenario_dict = record_result['scenario']
    # scenario = ScenarioObj()
    # scenario.load_from_json(scenario_dict)
    rlaga.run()
    # rlaga.reload_scenario(scenario)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down marker spawner processor...")
    except rospy.ROSException as e:
        if rospy.is_shutdown():
            pass
        else:
            rospy.logerr(e)