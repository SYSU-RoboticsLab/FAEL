#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

import rospy
from visualization_tools.msg import ExploredVolumeTravedDistTime
from visualization_tools.msg import IterationTime

mpl.rcParams['toolbar'] = 'None'
plt.ion()

time_duration1 = 0
time_duration2 = 0
time_duration3 = 0
time_duration4 = 0

explored_volume = 0
traveling_distance = 0
run_time = 0
max_explored_volume = 0
max_traveling_distance = 0
max_run_time = 0

time_list1 = np.array([])
time_list2 = np.array([])
time_list3 = np.array([])
time_list4 = np.array([])
explored_volume_list = np.array([])
traveling_distance_list = np.array([])
run_time_list = np.array([])


def exploration_data_callback(msg):
    global time_duration1, time_duration2, explored_volume, traveling_distance
    time_duration1 = msg.timeConsumed
    time_duration2 = msg.timeConsumed
    explored_volume = msg.exploredVolume
    traveling_distance = msg.travedDist


def iteration_time_callback(msg):
    global time_duration3, run_time, time_list3, run_time_list, max_run_time
    run_time = msg.iterationTime
    time_duration3 = msg.timeConsumed
    time_list3 = np.append(time_list3, time_duration3)
    run_time_list = np.append(run_time_list, run_time)
    if run_time > max_run_time:
        max_run_time = run_time


def listener():
    global explored_volume, traveling_distance, run_time, max_explored_volume, max_traveling_distance, max_run_time, time_list1, time_list2, time_list3, run_time_list, explored_volume_list, traveling_distance_list

    rospy.init_node('realTimePlot')

    rospy.Subscriber("/explored_volume_traved_dist_time",
                     ExploredVolumeTravedDistTime, exploration_data_callback)
    rospy.Subscriber("/run_time", IterationTime, iteration_time_callback)

    fig = plt.figure(figsize=(10, 8))
    plt.suptitle("Exploration Metrics\n", fontsize=18)
    plt.subplots_adjust(wspace=0.3, hspace=0.3)

    fig1 = fig.add_subplot(221)
    fig1.set_title('Explored Volume')
    plt.margins(x=0.0001, y=0.0001)
    fig1.set_ylabel("Explored\nVolume (m$^2$)", fontsize=8)
    fig1.set_xlabel("Time (s)", fontsize=8)
    l1, = fig1.plot(time_list1,
                    explored_volume_list,
                    color='r',
                    label='Explored Volume')

    fig2 = fig.add_subplot(222)
    fig2.set_title('Traveling Distance')
    fig2.set_ylabel("Traveling\nDistance (m)", fontsize=8)
    fig2.set_xlabel("Time (s)", fontsize=8)
    l2, = fig2.plot(time_list2,
                    traveling_distance_list,
                    color='r',
                    label='Traveling Distance')

    fig3 = fig.add_subplot(223)
    fig3.set_title('Volume Distance')
    fig3.set_ylabel("Explored\nVolume (m$^2$)", fontsize=8)
    fig3.set_xlabel("Traveling Distance (m)", fontsize=8)  # only set once
    l3, = fig3.plot(traveling_distance_list,
                    explored_volume_list,
                    color='r',
                    label='Volume Distance')

    fig4 = fig.add_subplot(224)
    fig4.set_title('Algorithm Runtime')
    fig4.set_ylabel("Algorithm\nRuntime (s)", fontsize=8)
    fig4.set_xlabel("Time (s)", fontsize=8)  # only set once
    l4, = fig4.plot(time_list3,
                    run_time_list,
                    'o',
                    color='r',
                    label='Algorithm Runtime')

    count = 0
    r = rospy.Rate(100)  # 100hz
    while not rospy.is_shutdown():
        r.sleep()
        count = count + 1

        if count % 10 == 0:
            max_explored_volume = explored_volume
            max_traveling_distance = traveling_distance

            time_list1 = np.append(time_list1, time_duration1)
            explored_volume_list = np.append(explored_volume_list,
                                             explored_volume)
            time_list2 = np.append(time_list2, time_duration2)
            traveling_distance_list = np.append(traveling_distance_list,
                                                traveling_distance)

        if count >= 100:
            count = 0
            l1.set_xdata(time_list1)
            l2.set_xdata(time_list2)
            l3.set_xdata(traveling_distance_list)
            l4.set_xdata(time_list3)
            l1.set_ydata(explored_volume_list)
            l2.set_ydata(traveling_distance_list)
            l3.set_ydata(explored_volume_list)
            l4.set_ydata(run_time_list)

            fig1.set_ylim(0, max_explored_volume + 100)
            fig1.set_xlim(0.2, time_duration1 + 10)
            fig2.set_ylim(0, max_traveling_distance + 20)
            fig2.set_xlim(0.2, time_duration2 + 10)
            fig3.set_ylim(10, max_explored_volume + 100)
            fig3.set_xlim(0.5, max_traveling_distance + 20)
            fig4.set_ylim(0, max_run_time + 0.01)
            fig4.set_xlim(0, time_duration3 + 10)

            fig.canvas.draw()


if __name__ == '__main__':
    listener()
    print("1")
