import rosbag
import glob
import os
import time
import math
import matplotlib.pyplot as plt
import numpy as np
import re

text_filter = "joint1_kp2_target2800"

def identify_bags():
    path = '/home/alex/.ros/'
    file_name = '*.bag'
    print(glob.glob(path + file_name))
    return glob.glob(path + file_name)

def read_bag(bag_file):
    bag = rosbag.Bag(bag_file)

    x_positions=[]
    y_positions=[]
    times = []
    first_loop=1

    # TODO: update to X and Y once it is implemented
    # for topic, msg, t in bag.read_messages(topics=['actual_position']):
    for topic, msg, t in bag.read_messages():

        # store data in positions
        x_positions.append(msg.data)
        y_positions.append(msg.data)

        # convert rosbag time variable to relative time
        # store relative times in times
        if first_loop:
            start_t = t
            first_loop = 0
        times.append((t - start_t).to_sec())

    bag.close()

    return x_positions, y_positions, times

    '''
    msgs = bag.read_messages(topics='actual_position')
    for msg in msgs:
        print(msg.topic)
        print(msg.message.data)
        print(msg.timestamp)
    '''

def create_path_plot(x_positions, y_positions, times):
    # plt.ion()
    # d = DynamicUpdate()
    # d(x_positions, y_positions, times)
    # plt.ioff()
    # input('Press enter to continue...')
    # os.system('read -s -n 1 -p "Press any key to continue..."')
    xpos_np = np.array(x_positions)
    times_np = np.array(times)
    mask = xpos_np > 100


    xpos_np = xpos_np[mask]
    times_np = times_np[mask]
    
    fig, ax = plt.subplots()
    ax.plot(times_np, xpos_np,'o-')
    ax.grid()
    plt.show()

class DynamicUpdate():

    def on_launch(self):
        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([],[], 'o')
        #Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscalex_on(True)
        self.ax.set_autoscaley_on(True)
        # self.ax.set_xlim(self.min_x, self.max_x)
        #Other stuff
        self.ax.grid()
        ...

    def on_running(self, xdata, ydata):
        #Update data (with the new _and_ the old points)
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    #Example
    def __call__(self, xdata, ydata, times):
        self.on_launch()
        
        
        for i in range(1,len(x_positions)):
            # tic = time.time()
            self.on_running(xdata[:i], ydata[:i])
            time.sleep(times[i]-times[i-1])
            # toc = time.time()
            # print(toc-tic-(times[i]-times[i-1]))



if __name__ == "__main__":
    bag_files = identify_bags()
    x_positions, y_positions, times = read_bag(bag_files[0])
    create_path_plot(x_positions,[math.sqrt(i) for i in y_positions],times)
    