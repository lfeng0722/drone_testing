import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')
import pygad


run_score_list1 = np.loadtxt('1obj')
run_score_list2 = np.loadtxt('3obj')
run_score_list4 = np.loadtxt('4obj')
plt.plot(run_score_list1,'x')
plt.plot(run_score_list2,'o')
plt.plot(run_score_list4,'1')

# axes = plt.axes()
# axes.set_ylim([0, 100])
plt.show()