# !/usr/bin/python3

import os
import matplotlib.pyplot as plt


def read_txt():
    explored_time = []
    distance = []
    explored_volume = []
    
    with open('time_distance_volume.txt', 'r') as f:
        lines = f.readlines()
        lines = lines[1:]

        for line in lines:

            time, dis, vol, _, _ = line.split('\t')
            explored_time.append(float(time))
            distance.append(float(dis))
            explored_volume.append(float(vol))


    plt.plot(explored_time, distance,color='green')
    plt.xlabel('time')
    plt.ylabel('distance')
    plt.show()

if __name__ == '__main__':
    read_txt()

        

