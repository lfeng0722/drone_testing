#!/usr/bin/env python3

from test_generation.ga.ga import GA
import rospy
from airsim_simulation.sim_env import AirSimEnv


if __name__ == '__main__':
    rospy.init_node('rlaga_search', anonymous=True)
    # rospy.Rate(1)
    rospy.loginfo("Starting rlaga_search processor...")

    env = AirSimEnv(host_ip='10.6.37.180')
    pop_size = 20
    generations = 20
    mutation_rate = 0.2
    scenario_topic = '/simulation/scenario'
    ga = GA(pop_size, generations, mutation_rate, env, scenario_topic)
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