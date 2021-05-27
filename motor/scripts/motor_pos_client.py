#!/usr/bin/env python3
 
import time
import rospy
from motor.srv import *
from motor.msg import *

m1_range = [1200, 2800]
DXL_ID1                     = 0                 # Motor For Joint 1: 1
DXL_ID2                     = 1                 # Motor for Joint 2: 2
DXL_ID3                     = 2                 # Motor for Joint 3: 3
# MOTOR_IDs = [DXL_ID1, DXL_ID2, DXL_ID3]         #
MOTOR_IDs = [DXL_ID1, DXL_ID2]         #
    
def read_motor_pos():
    #declare ServiceProxy to read the motor position
    rospy.wait_for_service('get_position')
    motor_pos = rospy.ServiceProxy('get_position', GetMotorPosition)
    try:
        # attempt to read motor positions. returns an array of positions.
        # return [motor_pos(DXL_ID1).position, motor_pos(DXL_ID2).position, motor_pos(DXL_ID3).position]
        return [motor_pos(DXL_ID1).position, motor_pos(DXL_ID2).position]

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    #declare publisher to write motor position
    rospy.init_node('motor_pos_writer')
    motor_writer = rospy.Publisher('set_position', SetPosition, queue_size=10)

    #decalares a target position variable of SetPosition data type (msg)
    target_pos = SetPosition()
    print('MotorWriter Initialized...')

    #set motors to min positions
    rate = rospy.Rate(.25)
    for (m_id,m_pos) in zip(MOTOR_IDs,[1200, 1200]):
        target_pos.id = m_id
        target_pos.position = m_pos
        print(f'{target_pos}')
        motor_writer.publish(target_pos)
    rate.sleep()

    #set the command rate in Hz
    rate = rospy.Rate(10)
    positions = []
    n = 200
    tic = time.perf_counter()

    for i in range(n):
        #save current position
        positions.append(read_motor_pos())
        print(positions[i])

        #move to new position
        #loop will fail if the motor IDs are changed
        for (m_id,m_pos) in zip(MOTOR_IDs,positions[i]):
            target_pos.id = m_id
            target_pos.position = m_pos+5
            print(f'{target_pos}')

            #check if motor is within its range of motion
            if target_pos.position < m1_range[0] or target_pos.position > m1_range[1]:
                #reset to middle of range of motion if out of bound
                target_pos.position = m1_range[0]

            motor_writer.publish(target_pos)
        
        rate.sleep()


    toc = time.perf_counter()
    print(f"Motor Pos Sampling Rate: {n/(toc - tic):0.4f} Hz")
    # print(positions)



# rostopic pub command for reference
# rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id:1, position: 0}"