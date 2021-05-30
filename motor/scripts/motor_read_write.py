#!/usr/bin/env python3
import rospy
import os, sys
# currentdir = os.path.dirname(os.path.realpath(__file__))
# parentdir = os.path.dirname(currentdir)
# sys.path.append(parentdir)

from dynamixel_sdk import *
from motor.srv import *
from motor.msg import *
import time


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

# Motor Clamping Value
MOTOR_CLAMP             = 440              # CLAMP PWM FULL SCALE [-885, 885]

# Control table address
ADDR_BAUD_RATE          = 8                # $
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116              # [0, 4095]
ADDR_GOAL_PWM           = 100              # [-885, 885]
ADDR_PRESENT_POSITION   = 132              # 360DEG SCALED TO [0, 4095]
ADDR_PRESENT_VELOCITY   = 128
ADDR_OPERATING_MODE     = 11               # 1 VELOCITY / 3 POSITION / 4 EXT POSITION / 16 PWM
ADDR_PRESENT_VELOCITY   = 128              # [REV / MIN]

LEN_POSITION                = 4            # Data Byte Length
LEN_PWM                     = 2            # Data Byte Length
LEN_VELOCITY                = 4            # Data Byte Length

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

# Initialize Port / Packet Handlers
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
# Initialize GroupBulkWrite instance
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
# Initialize GroupBulkRead instances
groupBulkReadPos = GroupBulkRead(portHandler, packetHandler)
groupBulkReadVel = GroupBulkRead(portHandler, packetHandler)

def set_goal_pos_callback(data):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, data.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, data.id, ADDR_OPERATING_MODE, 3)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, data.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    print("Set Goal Position of ID %s = %s" % (data.id, data.position))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)
    time.sleep(2)

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, data.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, data.id, ADDR_OPERATING_MODE, 16)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, data.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)


def set_motor_pwm_callback(data):
    print("Set Goal PWM of ID %s = %s" % (data.id, data.pwm))
    if data.pwm > MOTOR_CLAMP:
        data.pwm = MOTOR_CLAMP
    if data.pwm < -MOTOR_CLAMP:
        data.pwm = -MOTOR_CLAMP
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, data.id, ADDR_GOAL_PWM, data.pwm)

def bulk_set_pwm_callback(data):
    #Convert PWM Values to Byte Strings
    data_write1 = [DXL_LOBYTE(data.value1), DXL_HIBYTE(data.value1)]
    data_write2 = [DXL_LOBYTE(data.value2), DXL_HIBYTE(data.value2)]
    data_write3 = [DXL_LOBYTE(data.value3), DXL_HIBYTE(data.value3)]

    # Add Dynamixel#1 goal pwm to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(data.id1, ADDR_GOAL_PWM, LEN_PWM, data_write1)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkWrite addparam failed" % data.id1)
        quit()
    dxl_addparam_result = groupBulkWrite.addParam(data.id2, ADDR_GOAL_PWM, LEN_PWM, data_write2)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkWrite addparam failed" % data.id1)
        quit()
    dxl_addparam_result = groupBulkWrite.addParam(data.id3, ADDR_GOAL_PWM, LEN_PWM, data_write3)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkWrite addparam failed" % data.id1)
        quit()

    # Bulkwrite
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()

def get_present_pos(req):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
    return dxl_present_position


def bulk_read_pos_init():
    dxl_addparam_result = groupBulkReadPos.addParam(DXL_ID1, ADDR_PRESENT_POSITION, LEN_POSITION)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkReadPos addparam failed" % DXL_ID1)
        quit()
    dxl_addparam_result = groupBulkReadPos.addParam(DXL_ID2, ADDR_PRESENT_POSITION, LEN_POSITION)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkReadPos addparam failed" % DXL_ID2)
        quit()
    dxl_addparam_result = groupBulkReadPos.addParam(DXL_ID3, ADDR_PRESENT_POSITION, LEN_POSITION)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkReadPos addparam failed" % DXL_ID3)
        quit()

def bulk_get_pos(req):
    # Bulkread present position and LED status
    dxl_comm_result = groupBulkReadPos.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Check if groupbulkread data of Dynamixel#1 is available
    dxl_getdata_result = groupBulkReadPos.isAvailable(DXL_ID1, ADDR_PRESENT_POSITION, LEN_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkReadPos getdata failed" % DXL_ID1)
        quit()
    # Check if groupbulkread data of Dynamixel#2 is available
    dxl_getdata_result = groupBulkReadPos.isAvailable(DXL_ID2, ADDR_PRESENT_POSITION, LEN_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkReadPos getdata failed" % DXL_ID2)
        quit()
    # Check if groupbulkread data of Dynamixel#3 is available
    dxl_getdata_result = groupBulkReadPos.isAvailable(DXL_ID3, ADDR_PRESENT_POSITION, LEN_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkReadPos getdata failed" % DXL_ID3)
        quit()

    # Get present position value
    resp = BulkGetResponse()
    resp.value1 = groupBulkReadPos.getData(DXL_ID1, ADDR_PRESENT_POSITION, LEN_POSITION)
    resp.value2 = groupBulkReadPos.getData(DXL_ID2, ADDR_PRESENT_POSITION, LEN_POSITION)
    resp.value3 = groupBulkReadPos.getData(DXL_ID3, ADDR_PRESENT_POSITION, LEN_POSITION)

    return resp

def bulk_read_vel_init():
    dxl_addparam_result = groupBulkReadPos.addParam(DXL_ID1, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkReadVel addparam failed" % DXL_ID1)
        quit()
    dxl_addparam_result = groupBulkReadPos.addParam(DXL_ID2, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkReadVel addparam failed" % DXL_ID2)
        quit()
    dxl_addparam_result = groupBulkReadPos.addParam(DXL_ID3, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkReadVel addparam failed" % DXL_ID3)
        quit()

def bulk_get_vel(req):
    # Bulkread present position and LED status
    dxl_comm_result = groupBulkReadPos.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Check if groupbulkread data of Dynamixel#1 is available
    dxl_getdata_result = groupBulkReadPos.isAvailable(DXL_ID1, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkReadVel getdata failed" % DXL_ID1)
        quit()
    # Check if groupbulkread data of Dynamixel#2 is available
    dxl_getdata_result = groupBulkReadPos.isAvailable(DXL_ID2, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkReadVel getdata failed" % DXL_ID2)
        quit()
    # Check if groupbulkread data of Dynamixel#3 is available
    dxl_getdata_result = groupBulkReadPos.isAvailable(DXL_ID3, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkReadVel getdata failed" % DXL_ID3)
        quit()

    # Get present position value
    resp = BulkGetResponse()
    resp.value1 = groupBulkReadPos.getData(DXL_ID1, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)
    resp.value2 = groupBulkReadPos.getData(DXL_ID2, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)
    resp.value3 = groupBulkReadPos.getData(DXL_ID3, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)

    return resp


def read_write_py_node():
    print('starting node init')
    rospy.init_node('read_write_py_node')

    #Intitialize subscribers to publish motor values
    rospy.Subscriber('set_motor_position', SetMotorPosition, set_goal_pos_callback)
    rospy.Subscriber('set_motor_pwm', SetMotorPWM, set_motor_pwm_callback)
    rospy.Subscriber('bulk_set_pwm', BulkSetPWM, bulk_set_pwm_callback)

    #Initialize services to read motor values
    rospy.Service('get_motor_position', GetMotorPosition, get_present_pos)
    rospy.Service('bulk_get_position', BulkGet, bulk_get_pos)
    rospy.Service('bulk_get_velocity', BulkGet, bulk_get_vel)

    print('node init compelted. starting spin...')
    rospy.spin()

def motor_shutdown():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    time.sleep(.1)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    time.sleep(.1)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    time.sleep(.1)

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

    # Enable Dynamixel Torque and Set to PWM control
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
        # Disable Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        time.sleep(.1)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        time.sleep(.1)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        time.sleep(.1)

        # Write motors to PWM control
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_OPERATING_MODE, 16)
        time.sleep(.1)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_OPERATING_MODE, 16)
        time.sleep(.1)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID3, ADDR_OPERATING_MODE, 16)
        time.sleep(.1)

        # Enable Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        time.sleep(.1)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        time.sleep(.1)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        time.sleep(.1)
        print("DYNAMIXEL has been successfully connected")


    print("Ready to get & set Position.")
    bulk_read_pos_init()
    read_write_py_node()
    # bulk_read_vel_init()



if __name__ == '__main__':
    main()

