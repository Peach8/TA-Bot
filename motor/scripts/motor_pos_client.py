#!/usr/bin/env python3
 
import time
import rospy
from motor.srv import *
from motor.msg import *
import random

m_range = [1400, 2600]
DXL_ID1                     = 0                 # Motor For Joint 1: 1
DXL_ID2                     = 1                 # Motor for Joint 2: 2
DXL_ID3                     = 2                 # Motor for Joint 3: 3
# MOTOR_IDs = [DXL_ID1, DXL_ID2, DXL_ID3]         #
MOTOR_IDs = [DXL_ID1, DXL_ID2]         #
    
def read_motor_pos():
    #declare ServiceProxy to read the motor position
    rospy.wait_for_service('bulk_get_position')
    bulk_motor_pos = rospy.ServiceProxy('bulk_get_position', BulkGet)
    try:
        bulk_pos_response = bulk_motor_pos()
        return [bulk_pos_response.value1, bulk_pos_response.value2, bulk_pos_response.value3]

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    #declare publisher to write motor position
    rospy.init_node('motor_pwm_writer')
    motor_writer = rospy.Publisher('bulk_set_pwm', BulkSetPWM, queue_size=10)

    #decalares a target position variable of SetPosition data type (msg)
    target_pwm = BulkSetPWM()
    print('MotorWriter Initialized...')

    #set the command rate in Hz
    rate = rospy.Rate(100)
    positions = []
    directions = [1, 1]
    n = 1000
    tic = time.perf_counter()

    for i in range(n):
        #save current position
        positions.append(read_motor_pos())
        print(positions[i])

        # move to new position
        # loop will fail if the motor IDs are changed

        for (m_id,m_pos) in zip(MOTOR_IDs,positions[i]):
            if m_pos < m_range[0]:
                directions[m_id] = 1
            elif m_pos > m_range[1]:
                directions[m_id] = -1
            
        target_pwm.id1 = DXL_ID1
        target_pwm.id2 = DXL_ID2
        target_pwm.id3 = DXL_ID3
        pwm_val = pwm_val = random.randint(30, 200)
        target_pwm.value1 = directions[DXL_ID1]*pwm_val
        target_pwm.value2 = directions[DXL_ID2]*pwm_val
        target_pwm.value3 = 0
        print([target_pwm.value1,target_pwm.value2,target_pwm.value3])
        motor_writer.publish(target_pwm)
        
        rate.sleep()


    toc = time.perf_counter()
    print(f"Motor Pos Sampling Rate: {n/(toc - tic):0.4f} Hz")

    target_pwm.id1 = DXL_ID1
    target_pwm.id2 = DXL_ID2
    target_pwm.id3 = DXL_ID3
    target_pwm.value1 = 0
    target_pwm.value2 = 0
    target_pwm.value3 = 0

    motor_writer.publish(target_pwm)
        


    # print(positions)



# rostopic pub command for reference
# rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id:1, position: 0}"