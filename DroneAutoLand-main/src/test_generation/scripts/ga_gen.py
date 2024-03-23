#!/usr/bin/env python3

from test_generation.ga.ga import GA
import rospy
from airsim_simulation.sim_env import AirSimEnv


if __name__ == '__main__':
    rospy.init_node('rlaga_search', anonymous=True)
    # rospy.Rate(1)
    rospy.loginfo("Starting rlaga_search processor...")

    host_ip='127.0.0.1'
    pop_size = 10
    generations = 21
    mutation_rate = 0.2
    scenario_topic = '/simulation/scenario'
    result_save_path = '//media/linfeng/HDD1/NEW_ICSE/GA_MM_Lawn_02'
    map_name = 'lawn'
    ga = GA(pop_size, generations, mutation_rate, host_ip, scenario_topic, map_name, result_save_path=result_save_path)
    ga.run()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down marker spawner processor...")
    except rospy.ROSException as e:
        if rospy.is_shutdown():
            pass
        else:
            rospy.logerr(e)