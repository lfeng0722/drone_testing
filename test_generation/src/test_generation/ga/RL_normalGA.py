from test_generation.ga.ga import GA, start_scenario, scenario_to_msg
import rospy
import time
from airsim_simulation.sim_env import AirSimEnv
import numpy as np
import random

def cal_fitness(scenario_result, scenario_rewards=None):


    violation_obj=[]
    landing_time = []
    fp_obj = []

    for item in scenario_result:
        violation_obj.append(item['land_deviation'])
        landing_time.append(item['land_duration'])
        fp_obj.append(item['land_fp_dist'])


    violation_arr = np.where(np.array(violation_obj) > 1.5, 1, 0)

    landing_time_min = np.min(landing_time)
    landing_time_max = np.max(landing_time)
    landing_time_arr = (landing_time - landing_time_min) / (landing_time_max - landing_time_min)


    fp_arr = np.where(np.array(fp_obj) < 1.5, 1, 0)

    fitness_Value = violation_arr + landing_time_arr + fp_arr

    return fitness_Value

def get_state(env, scenario, npc_idx):
    marker_position = (scenario.tp_marker.pose.x, scenario.tp_marker.pose.y)
    actor_pose = env.get_pose(env.dynamic_objects[npc_idx])
    actor_position = (actor_pose.position.x_val, actor_pose.position.y_val)
    drone_current_pose = env.client.simGetVehiclePose()
    drone_current_position = [drone_current_pose.position.x_val, drone_current_pose.position.y_val]

    state  = (actor_position[0]-marker_position[0],actor_position[1]-marker_position[1],
                drone_current_position[0]-marker_position[0],drone_current_position[1]-marker_position[1])
    
    return state

class RLAGA(GA):
    def __init__(self, rl_agent, pop_size, generations, mutation_rate, env, scenario_topic, result_save_path='/home/linfeng/Documents/rlaga_result'):
        super(RLAGA, self).__init__(pop_size, generations, mutation_rate, env, scenario_topic, result_save_path)
        self.rl_agent = rl_agent

    def reload_scenario(self, scenario):
        start_scenario(scenario, self.env)
        scenario_msg = scenario_to_msg(scenario)
        # scenario_msg = scenario.to_msg()
        scenario_msg.header.stamp = rospy.Time.now()

        self.scenario_pub.publish(scenario_msg)
        print('publish scenario')
        time.sleep(5)
        agents = [self.rl_agent(4,[0, 1, 2, 3, 4]) for _ in range(len(self.env.dynamic_objects))]

        # self.env.run_scenario()
        while rospy.is_shutdown() is False:
            idx = 0
            while rospy.is_shutdown() is False:
                if idx == len(self.env.dynamic_objects):
                    break
                if self.scenario_status == 'end':
                    break

                actor = self.env.dynamic_objects[idx]
                agent_state = get_state(self.env, scenario, idx)
                action = agents[idx].evaluate(agent_state)
                print(actor, action)
                if action == 1 and self.scenario_status != 'end':
                    self.env.move_npc_forward(actor, 1)
                elif action == 2  and self.scenario_status != 'end':
                    self.env.move_npc_backward(actor, 1)
                elif action == 3  and self.scenario_status != 'end':
                    self.env.move_npc_left(actor, 1)
                elif action == 4  and self.scenario_status != 'end':
                    self.env.move_npc_right(actor, 1)

                idx += 1


            if self.scenario_status == 'end':
                # fitness_values.append(cal_fitness(self.current_scenario_result))
                rospy.loginfo('finish scenario. save scenario result')
                self.generation_result.append(self.current_scenario_result.get_result())
                # self.save_scenario_result(current_generation, current_scenario_id)
                time.sleep(3)
                self.reset_scenario_data()
                break

            time.sleep(2)

            # while rospy.is_shutdown() is False:
            # drone_pose = self.env.client.simGetVehiclePose()
            # if abs(drone_pose.position.x_val) < 0.3 and abs(drone_pose.position.y_val) < 0.3 and \
            #     self.scenario_status == 'end':
            # time.sleep(3)
            # print('4444444444444444444444')
            # break

            # rate.sleep()
            time.sleep(1)

    def run(self):
        rospy.loginfo('start RLAGA test generation')

        population = self.initial_population()
        # print('1111111111111111111',population[0])
        current_generation = 0
        while rospy.is_shutdown() is False:
            if current_generation == self.generations - 1:
                
                break

            current_scenario_id = 0
            while rospy.is_shutdown() is False:
                if current_scenario_id == len(population):
                    actor_pool=[]
                    actor_fitness_sample_weight = []
                    fitness_values = cal_fitness(self.generation_result)
                    element_to_weight = dict(zip(population, fitness_values))
                    parents = random.choices(population, weights=fitness_values, k=self.pop_size)
                    sample_weights = [element_to_weight[sample] for sample in parents]

                    for i in range(0, len(population)):
                        if i < len(actor_pool) - 1 and i % 2 == 0:
                            chromosome1 = population[i]
                            chromosome2 = population[i + 1] if i + 1 < len(population) else None

                            if chromosome1 != None and chromosome2 != None:
                                chromosome1.weather.crossover(chromosome1.weather, chromosome2.weather)
                                chromosome1.tp_marker.crossover(chromosome1.tp_marker, chromosome2.tp_marker)
                                chromosome1.fp_markers[0].crossover(chromosome1.fp_markers[0], chromosome2.fp_markers[0])
                                chromosome1.fp_markers[1].crossover(chromosome1.fp_markers[1], chromosome2.fp_markers[1])

                        population[i].weather.mutate()
                        population[i].tp_marker.mutate()
                        population[i].fp_markers[0].mutate()
                        population[i].fp_markers[1].mutate()

                    for chromo, weight in zip(population, sample_weights):
                        for actor in chromo.actors:
                            actor_pool.append(actor)
                            actor_fitness_sample_weight.append(weight)
                    for i in range(0, len(actor_pool)):
                        if i < len(actor_pool) - 1 and i % 2 == 0:
                            actor1 = actor_pool[i]
                            actor2 = actor_pool[i + 1]
                            actor1.crossover(actor1, actor2)
                        actor_pool[i].mutate()
                    # population = self.initial_population()

                    current_generation += 1
                    self.generation_result = []
                    
                    break

                rospy.loginfo("start exp:{}-{}".format(current_generation+1, current_scenario_id+1))
                scenario = population[current_scenario_id]
                current_scenario_id += 1

                start_scenario(scenario, self.env)
                scenario_msg = scenario_to_msg(scenario)
                # scenario_msg = scenario.to_msg()
                scenario_msg.header.stamp = rospy.Time.now()

                self.scenario_pub.publish(scenario_msg)
                print('publish scenario')
                time.sleep(5)
                agents = [self.rl_agent(4,[0, 1, 2, 3, 4]) for _ in range(len(self.env.dynamic_objects))]

                # self.env.run_scenario()
                while rospy.is_shutdown() is False:
                    idx = 0
                    while rospy.is_shutdown() is False:
                        if idx == len(self.env.dynamic_objects):
                            break
                        if self.scenario_status == 'end':
                            break

                        actor = self.env.dynamic_objects[idx]
                        agent_state = get_state(self.env, scenario, idx)
                        action = agents[idx].evaluate(agent_state)
                        print(actor, action)
                        if action == 1 and self.scenario_status != 'end':
                            self.env.move_npc_forward(actor, 1)
                        elif action == 2  and self.scenario_status != 'end':
                            self.env.move_npc_backward(actor, 1)
                        elif action == 3  and self.scenario_status != 'end':
                            self.env.move_npc_left(actor, 1)
                        elif action == 4  and self.scenario_status != 'end':
                            self.env.move_npc_right(actor, 1)
                        
                        idx += 1


                    if self.scenario_status == 'end':
                        # fitness_values.append(cal_fitness(self.current_scenario_result))
                        rospy.loginfo('finish scenario. save scenario result')
                        self.generation_result.append(self.current_scenario_result.get_result())
                        self.save_scenario_result(current_generation, current_scenario_id)
                        time.sleep(3)
                        self.reset_scenario_data()
                        break
                        
                    time.sleep(2)
                
                # while rospy.is_shutdown() is False:   
                    # drone_pose = self.env.client.simGetVehiclePose()
                    # if abs(drone_pose.position.x_val) < 0.3 and abs(drone_pose.position.y_val) < 0.3 and \
                    #     self.scenario_status == 'end':
                        # time.sleep(3)
                        # print('4444444444444444444444')
                        # break

                    # rate.sleep()
                    time.sleep(1)
