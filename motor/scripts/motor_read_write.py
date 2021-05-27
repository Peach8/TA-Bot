#!/usr/bin/env python3
import rospy
import os, sys
# currentdir = os.path.dirname(os.path.realpath(__file__))
# parentdir = os.path.dirname(currentdir)
# sys.path.append(parentdir)

from dynamixel_sdk import *
from motor.srv import *
from motor.msg import *


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


# Control table address
ADDR_BAUD_RATE          = 8                # $
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116              # [0, 4095]
ADDR_GOAL_PWM           = 100              # [-885, 885]
ADDR_PRESENT_POSITION   = 132              # 360DEG SCALED TO [0, 4095]
ADDR_OPERATING_MODE     = 11               # 1 VELOCITY / 3 POSITION / 4 EXT POSITION / 16 PWM

LEN_PRESENT_POSITION        = 4         # Data Byte Length
LEN_GOAL_POSITION           = 4         # Data Byte Length


# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID1                     = 0                 # Motor For Joint 1: 0
DXL_ID2                     = 1                 # Motor for Joint 2: 1
DXL_ID3                     = 2                 # Motor for Joint 3: 2
BAUDRATE                    = 3000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
# bulkReader = GroupBulkRead(portHandler,packetHandler)

def set_goal_pos_callback(data):
    print("Set Goal Position of ID %s = %s" % (data.id, data.position))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)

def set_motor_pwm_callback(data):
    print("Set Goal PWM of ID %s = %s" % (data.id, data.pwm))

    if data.pwm > 885: data.pwm = 885
    if data.pwm < -885: data.pwm = -885
    
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, data.id, ADDR_GOAL_PWM, data.pwm)

def get_present_pos(req):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
    return dxl_present_position

# def get_preset_pos_bulk(req):
#     return dxl_present_positions


def read_write_py_node():
    rospy.init_node('read_write_py_node')
    rospy.Subscriber('set_position', SetPosition, set_goal_pos_callback)
    rospy.Subscriber('set_PWM', SetMotorPWM, set_motor_pwm_callback)
    rospy.Service('get_position', GetMotorPosition, get_present_pos)
    rospy.spin()

def main():
    # Open port
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        # write motors to PWM control
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_OPERATING_MODE, 16)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_OPERATING_MODE, 16)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID3, ADDR_OPERATING_MODE, 16)

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        print("DYNAMIXEL has been successfully connected")

    print("Ready to get & set Position.")

    read_write_py_node()

if __name__ == '__main__':
    main()



''' code to read data
data, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE)
print(f'torque enaabled: {data}')
'''