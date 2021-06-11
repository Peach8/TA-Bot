import rosbag
import glob
import os
import time
import math
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
import re
from numpy.linalg import norm

# text_filter = ".*joint1_kp2.*"
text_filter = ".*"+"actual_position_100hz"+".*"


def identify_bags():
    path = '/home/alex/.ros/'
    file_name = '*.bag'
    # print(glob.glob(path + file_name))
    return glob.glob(path + file_name)


def read_bag(bag_file):
    bag = rosbag.Bag(bag_file)

    #FIRST READ ALL DATA INTO LIST FORMAT
    #CONVERTS TIME TO SECONDS FROM INITIAL TIMESTAMP
    topics = []
    msgs = []
    times = []
    first_loop=1

    for topic, msg, t in bag.read_messages():
        topics.append(topic)
        msgs.append(msg)

        if first_loop:
            start_t = t
            first_loop = 0
        times.append((t - start_t).to_sec())


    # INITIALIZE DICTIONARY WITH ALL ROSTOPICS
    topic_list = list(set(topics))
    # print(topic_list)

    data_dict = {topic: [] for topic in topic_list}
    # print(data_dict)

    # SAVE DATA TO A DICTIONARY WITH TOPIC AS KEYWORD
    # DATA SAVED IN TUPLE FORMAT: (DATA, TIMESTAMP)
    for (topic, msg, t) in zip(topics, msgs, times):
        data_dict[topic].append((msg.data,t))

    bag.close()

    return data_dict


def create_multi_plot(data_dict):
    start_time = -.1
    end_time = .4

    
    ## This section creates a 6 in 1 plot of the motors and x / y history
    def format_plots(fig):
        print(type(fig))
        for i,ax in enumerate(fig.get_axes()):
            ax.legend()
            ax.grid()
            ax.set_xlabel('time (sec)')


    fig1 = plt.figure(constrained_layout=True)
    gs = GridSpec(3, 1, figure=fig1)

    ax1 = fig1.add_subplot(gs[0, 0])
    ax2 = fig1.add_subplot(gs[1, 0])
    ax3 = fig1.add_subplot(gs[2, 0])
    fig1.suptitle(f"Individual Motor Tuning: Kp = 6.0, Kd = 0.1, Ki = 0.0, Pen Down")

    ## NOTE: 't' stores the timestamp in seconds, 'd' stores the data in the tuple
    ax1.plot([t for (d,t) in data_dict['actual_encoder1'] if (t > start_time and t < end_time)],[d for (d,t) in data_dict['actual_encoder1'] if (t > start_time and t < end_time)], marker='o', label='Encoder Position')
    ax1.axhline(2013,color='red',linestyle='dashed',label='Desired Position')
    ax1.set_title('Motor Encoder Space')
    ax1.set_ylabel('Encoder Position [0, 4095]')

    ax2.plot([t for (d,t) in data_dict['error_enc1'] if (t > start_time and t < end_time)],[-d for (d,t) in data_dict['error_enc1'] if (t > start_time and t < end_time)], marker='o',color='tab:orange',label='Encorder Error')
    ax2.set_title('Motor Encoder Error')
    ax2.set_ylabel('Encoder Error [0, 4095]')

    ax3.plot([t for (d,t) in data_dict['m1_pwm'] if (t > start_time and t < end_time)],[d for (d,t) in data_dict['m1_pwm'] if (t > start_time and t < end_time)], marker='o',label='PWM')
    ax3.set_title('Motor PWM')
    ax3.set_ylabel('PWM Value [-885, 885]')

    format_plots(fig1)

    ## Show the plots



# def create_path_plot(data_dict):
#     xpos_np = np.array([d for (d,t) in data_dict['actual_motor_position']])
#     times_np = np.array([t for (d,t) in data_dict['actual_motor_position']])
#     mask = (xpos_np > 100) * (times_np < 5)

#     xpos_np = xpos_np[mask]
#     times_np = times_np[mask]

#     # plt.ion()
#     # d = DynamicUpdate()
#     # d(times_np, xpos_np, times_np)
#     # plt.ioff()
#     # input('Press enter to continue...')
#     # # os.system('read -s -n 1 -p "Press any key to continue..."')
    
#     fig, ax = plt.subplots()
#     ax.plot(times_np, xpos_np,'o-')
#     ax.grid()
#     plt.show()

# class DynamicUpdate():

#     def on_launch(self):
#         #Set up plot
#         self.figure, self.ax = plt.subplots()
#         self.lines, = self.ax.plot([],[], 'o')
#         #Autoscale on unknown axis and known lims on the other
#         self.ax.set_autoscalex_on(True)
#         self.ax.set_autoscaley_on(True)
#         # self.ax.set_xlim(self.min_x, self.max_x)
#         #Other stuff
#         self.ax.grid()
#         ...

#     def on_running(self, xdata, ydata):
#         #Update data (with the new _and_ the old points)
#         self.lines.set_xdata(xdata)
#         self.lines.set_ydata(ydata)
#         #Need both of these in order to rescale
#         self.ax.relim()
#         self.ax.autoscale_view()
#         #We need to draw *and* flush
#         self.figure.canvas.draw()
#         self.figure.canvas.flush_events()

#     #Example
#     def __call__(self, xdata, ydata, times):
#         self.on_launch()
        
        
#         for i in range(1,len(x_positions)):
#             # tic = time.time()
#             self.on_running(xdata[:i], ydata[:i])
#             time.sleep(times[i]-times[i-1])
#             # toc = time.time()
#             # print(toc-tic-(times[i]-times[i-1]))



if __name__ == "__main__":
    bag_files = identify_bags()
    
    r = re.compile(text_filter)
    selected_bags = list(filter(r.match, bag_files))
    print(selected_bags)


    for bag in selected_bags:
        data_dict = read_bag(bag)
        # print(data_dict)
        print(data_dict.keys())
        create_multi_plot(data_dict)


    plt.show()
