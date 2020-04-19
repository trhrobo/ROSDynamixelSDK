#include<ros/ros.h>
#include<iostream>
#include<fcntl.h>
#include<termios.h>
#include<sensor_msgs/Joy.h>
#include<string>
#include<dynamixel_sdk.h>

using std::cout;
using std::endl;
using std::string;

//Control talble address
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

//Protocol version
#define PROTOCOL_VERSION 2.0

//Default setting
#define DXL_ID 1
#define BAUDRATE 57600
#define DEVICENAME "/dev/ttyUSB0"

//DXL parameter
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define MAX_POSITION_VALUE 1048575
#define MIN_POSITION_VALUE -1048575
#define DXL_MOVING_STATUS_THRESHOLD 20
#define EXT_POSITION_CONTROL_MODE 4

void msecSleep(int waitTime){
#if defined(_linux_)
    usleep(waitTime * 1000);
#endif
}

float joy_value{};
void joyCallback(const sensor_msgs::Joy &msg){
    joy_value = msg.axes[0];
}

namespace DXL{
    bool write1Byte(dynamixel::PortHandler *port, dynamixel::PacketHandler *packet, int addr, int value){
        uint8_t dxl_error{};
        int dxl_comm_result = COMM_TX_FAIL;
        dxl_comm_result = packet -> write1ByteTxRx(port, DXL_ID, addr, value, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS){
            string error = packet -> getTxRxResult(dxl_comm_result);
            cout << "error" << endl;
            return false;
        }else if(dxl_error != 0){
            string error = packet -> getRxPacketError(dxl_error);
            cout << "error" << endl;
            return false;
        }
        return true;
    }
    bool write4Byte(dynamixel::PortHandler *port, dynamixel::PacketHandler *packet, int addr, int value){
        uint8_t dxl_error{};
        int dxl_comm_result = COMM_TX_FAIL;
        dxl_comm_result = packet -> write4ByteTxRx(port, DXL_ID, addr, value, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS){
            string error = packet -> getTxRxResult(dxl_comm_result);
            cout << error << endl;
            return false;
        }else if(dxl_error != 0){
            string error = packet -> getRxPacketError(dxl_error);
            cout << error << endl;
            return false;
        }
        return true;
    }
    int32_t read4Byte(dynamixel::PortHandler *port, dynamixel::PacketHandler *packet, int addr){
        uint8_t dxl_error{};
        int32_t read_value{};
        int dxl_comm_result = COMM_TX_FAIL;
        dxl_comm_result = packet -> read4ByteTxRx(port, DXL_ID, addr, (uint32_t*)&read_value, &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS){
            string error = packet -> getTxRxResult(dxl_comm_result);
            cout << error << endl;
        }else if(dxl_error != 0){
            string error = packet -> getRxPacketError(dxl_error);
            cout << error << endl;
        }
        return read_value;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "DXL_Joy");
    ros::NodeHandle n;
    ros::Subscriber joy_sub = n.subscribe("joy", 10, joyCallback);
    ros::Rate loop_rate(100);
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error{};
    int32_t dxl_present_position{};

    //----------start port setting----------// 
    //Open port
    if(portHandler -> openPort()){
        cout << "Succeeded to open the port" << endl;
    }else{
        cout << "Failed to open the port" << endl;
        return 0;
    }

    //Set port baudrate
    if(portHandler -> setBaudRate(BAUDRATE)){
        cout << "Succeed to change the baudrate" << endl;
    }else{
        cout << "Failed to change the baudrate" << endl;
        return 0;
    }
    //----------finish port setting----------//

    //----------start packet setting----------//
    //Set operating mode to extended position control mode
    if(DXL::write1Byte(portHandler, packetHandler, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)){
        cout << "Operating mode changed to extended position control mode" << endl;
    }else{
        return 0;
    }

    if(DXL::write1Byte(portHandler, packetHandler, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)){
        cout << "Dynamixel has been successfully connected" << endl;
    }else{
        return 0;
    }
    //----------finish packet setting----------//

    while(ros::ok()){
        if(joy_value < -0.5 or 0.5 < joy_value){
            if(0.5 < joy_value){
                //Write goal position
                if(DXL::write4Byte(portHandler, packetHandler, ADDR_GOAL_POSITION, MAX_POSITION_VALUE) == false){
                    return 0;
                }
            }else{
                //Write goal position
                if(DXL::write4Byte(portHandler, packetHandler, ADDR_GOAL_POSITION, MIN_POSITION_VALUE) == false){
                    return 0;
                }
            }
        }else{
            //Read goal position
            dxl_present_position = DXL::read4Byte(portHandler, packetHandler, ADDR_PRESENT_POSITION);
            if(DXL::write4Byte(portHandler, packetHandler, ADDR_GOAL_POSITION, dxl_present_position) == false){
                return 0;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}