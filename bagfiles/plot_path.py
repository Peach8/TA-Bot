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
text_filter = ".*"+"93_presentation"+".*"


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

    if 'desired_x' not in topic_list:
        topic_list.append('desired_x')
        topic_list.append('desired_y')

    data_dict = {topic: [] for topic in topic_list}
    # print(data_dict)


    # SAVE DATA TO A DICTIONARY WITH TOPIC AS KEYWORD
    # DATA SAVED IN TUPLE FORMAT: (DATA, TIMESTAMP)
    for (topic, msg, t) in zip(topics, msgs, times):
        if topic == 'desired_point':
            data_dict['desired_x'].append((msg.x, t))
            data_dict['desired_y'].append((msg.y, t))
        else:
            data_dict[topic].append((msg.data,t))

    bag.close()

    return data_dict


def create_multi_plot(data_dict):
    start_time = data_dict['num1_start']
    # end_time = data_dict['num1_end']
    end_time = 10.86

    
    ## This section creates a 6 in 1 plot of the motors and x / y history
    def format_plots(fig):
        for i,ax in enumerate(fig.get_axes()):
            ax.legend()
            ax.grid()
            ax.set_xlabel('time (sec)')


    fig1 = plt.figure(constrained_layout=True)
    gs = GridSpec(3, 2, figure=fig1)

    ax1 = fig1.add_subplot(gs[0, 0])
    ax2 = fig1.add_subplot(gs[0, 1])
    ax3 = fig1.add_subplot(gs[1, 0])
    ax4 = fig1.add_subplot(gs[1, 1])
    ax5 = fig1.add_subplot(gs[2, 0])
    ax6 = fig1.add_subplot(gs[2, 1])
    fig1.suptitle(f"Task Space Results: {data_dict['file_name']}")

    ## NOTE: 't' stores the timestamp in seconds, 'd' stores the data in the tuple
    ax1.plot([t for (d,t) in data_dict['desired_encoder1'] if (t > start_time and t < end_time)],[d for (d,t) in data_dict['desired_encoder1'] if (t > start_time and t < end_time)],label='desired_encoder1', marker='o', markersize=3)
    ax1.plot([t for (d,t) in data_dict['actual_encoder1'] if (t > start_time and t < end_time)],[d for (d,t) in data_dict['actual_encoder1'] if (t > start_time and t < end_time)],label='actual_encoder1')
    ax1.set_title('Motor 1: Encoder Space')
    ax1.set_ylabel('dimensionless [0, 4095]')

    ax2.plot([t for (d,t) in data_dict['desired_encoder2'] if (t > start_time and t < end_time)],[d for (d,t) in data_dict['desired_encoder2'] if (t > start_time and t < end_time)],label='desired_encoder2', marker='o', markersize=3)
    ax2.plot([t for (d,t) in data_dict['actual_encoder2'] if (t > start_time and t < end_time)],[d for (d,t) in data_dict['actual_encoder2'] if (t > start_time and t < end_time)],label='actual_encoder2')
    ax2.set_title('Motor 2: Encoder Space')
    ax2.set_ylabel('dimensionless [0, 4095]')

    ax3.plot([t for (d,t) in data_dict['error_enc1'] if (t > start_time and t < end_time)],[-d for (d,t) in data_dict['error_enc1'] if (t > start_time and t < end_time)], color='tab:orange',label='error_enc1')
    ax3.set_title('Motor 1: Encoder Error')
    ax3.set_ylabel('dimensionless [0, 4095]')

    ax4.plot([t for (d,t) in data_dict['error_enc2'] if (t > start_time and t < end_time)],[-d for (d,t) in data_dict['error_enc2'] if (t > start_time and t < end_time)], color='tab:orange',label='error_enc2')
    ax4.set_title('Motor 2: Encoder Error')
    ax4.set_ylabel('dimensionless [0, 4095]')

    ax5.plot([t for (d,t) in data_dict['m1_pwm'] if (t > start_time and t < end_time)],[d for (d,t) in data_dict['m1_pwm'] if (t > start_time and t < end_time)],label='m1_pwm')
    ax5.set_title('Motor 1: PWM')
    ax5.set_ylabel('dimensionless [-885, 885]')

    ax6.plot([t for (d,t) in data_dict['m2_pwm'] if (t > start_time and t < end_time)],[d for (d,t) in data_dict['m2_pwm'] if (t > start_time and t < end_time)],label='m2_pwm')
    ax6.set_title('Motor 2: PWM')
    ax6.set_ylabel('dimensionless [-885, 885]')


    format_plots(fig1)


    fig2 = plt.figure(constrained_layout=True)
    gs2 = GridSpec(2, 1, figure=fig2)

    ax7 = fig2.add_subplot(gs2[0, 0])
    ax8 = fig2.add_subplot(gs2[1, 0])
    # ax9 = fig2.add_subplot(gs2[2, 0])
    fig2.suptitle(f"Task Space Results: {data_dict['file_name']}")


    ax7.plot([t for (d,t) in data_dict['desired_x'] if (t > start_time and t < end_time)],[d*1000 for (d,t) in data_dict['desired_x'] if (t > start_time and t < end_time)],label='desired_x', marker='o', markersize=3)
    ax7.plot([t for (d,t) in data_dict['actual_x'] if (t > start_time and t < end_time)],[d*1000 for (d,t) in data_dict['actual_x'] if (t > start_time and t < end_time)],label='actual_x')
    ax7.set_title('Task Space: X-History')
    ax7.set_ylabel('X (mm)')
    ax7.set_yticks(np.arange(100,190,10))

    ax8.plot([t for (d,t) in data_dict['desired_y'] if (t > start_time and t < end_time)],[d*1000 for (d,t) in data_dict['desired_y'] if (t > start_time and t < end_time)],label='desired_y', marker='o', markersize=3)
    ax8.plot([t for (d,t) in data_dict['actual_y'] if (t > start_time and t < end_time)],[d*1000 for (d,t) in data_dict['actual_y'] if (t > start_time and t < end_time)],label='actual_y')
    ax8.set_title('Task Space: Y-History')
    ax8.set_ylabel('Y (mm)')
    ax8.set_yticks(np.arange(200,290,10))

    # ax9.plot([t for (d,t) in data_dict['num1_error'] if (t > start_time and t < end_time)],[d*1000 for (d,t) in data_dict['num1_error'] if (t > start_time and t < end_time)],label='num1_error')
    # ax9.plot([t for (d,t) in data_dict['num2_error'] if (t > start_time and t < end_time)],[d*1000 for (d,t) in data_dict['num2_error'] if (t > start_time and t < end_time)],label='num2_error')
    # ax9.set_title('Task Space: Absoluter Error')


    format_plots(fig2)
    

    ## This section creates the XY actual vs desired plot
    fig2, ax = plt.subplots()
    ax.plot([d*1000 for (d,t) in data_dict['desired_x'] if (t > start_time and t < end_time)],[d*1000 for (d,t) in data_dict['desired_y'] if (t > start_time and t < end_time)],label='desired path', marker='o', markersize=3)
    ax.plot([d*1000 for (d,t) in data_dict['actual_x'] if (t > start_time and t < end_time)],[d*1000 for (d,t) in data_dict['actual_y'] if (t > start_time and t < end_time)],label='actual path')
    ax.set_title(f"Task Space: Actual vs Target Trajectory - {data_dict['file_name']}")
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.legend()
    ax.grid()


    ## Show the plots

def calc_min_error(data_dict):
    drawing_times = []
    previous_pos = False
    for (d,t) in data_dict['pen_down_pos']:
        if d != previous_pos:
            previous_pos = d
            drawing_times.append(t)

    print(f'drawing times: {drawing_times}')

    num1_start = drawing_times[0] - 0.1
    num1_end = drawing_times[1]
    num2_start = drawing_times[2] - 0.1
    num2_end = 1e6
    
    num1_error = []
    temp_min = 1000
    for ((act_x,act_tx),(act_y,act_ty)) in zip(data_dict['actual_x'],data_dict['actual_y']):
        for ((des_x,des_tx),(des_y,des_ty)) in zip(data_dict['desired_x'],data_dict['desired_y']):
            if act_tx > num1_start and act_tx < num1_end and des_tx > num1_start and des_tx < num1_end and norm([des_x-act_x,des_y-act_y]) < temp_min:
                temp_min = norm([des_x-act_x,des_y-act_y])
        if temp_min != 1000:
            num1_error.append((temp_min,act_tx))
            temp_min = 1000


    num2_error = []
    temp_min = 1000
    for ((act_x,act_tx),(act_y,act_ty)) in zip(data_dict['actual_x'],data_dict['actual_y']):
        for ((des_x,des_tx),(des_y,des_ty)) in zip(data_dict['desired_x'],data_dict['desired_y']):
            if act_tx > num2_start and act_tx < num2_end and des_tx > num2_start and des_tx < num2_end and norm([des_x-act_x,des_y-act_y]) < temp_min:
                temp_min = norm([des_x-act_x,des_y-act_y])
        if temp_min != 1000:
            num2_error.append((temp_min,act_tx))
            temp_min = 1000

    print(np.round([1000*np.average([d for (d,t) in num1_error]),1000*np.average([d for (d,t) in num1_error])],3))

    data_dict['num1_start'] = drawing_times[0]
    data_dict['num1_end'] = drawing_times[1]
    data_dict['num2_start'] = drawing_times[2]
    data_dict['num1_error'] = num1_error
    data_dict['num2_error'] = num2_error

    return data_dict

def create_path_plot(x_positions, y_positions, times):
    xpos_np = np.array(x_positions)
    times_np = np.array(times)
    mask = (xpos_np > 100) * (times_np < 5)

    xpos_np = xpos_np[mask]
    times_np = times_np[mask]

    # plt.ion()
    # d = DynamicUpdate()
    # d(times_np, xpos_np, times_np)
    # plt.ioff()
    # input('Press enter to continue...')
    # # os.system('read -s -n 1 -p "Press any key to continue..."')
    
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
    
    r = re.compile(text_filter)
    selected_bags = list(filter(r.match, bag_files))
    print(selected_bags)

    for bag in selected_bags:
        data_dict = read_bag(bag)
        data_dict['file_name'] = bag.split('/')[-1]
        print(data_dict.keys())
        data_dict = calc_min_error(data_dict)
        create_multi_plot(data_dict)



    plt.show()
