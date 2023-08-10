/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* This code has been adapted from "Read and Write Example" (whose autor is Ryu Woon Jung (Leon) and whose
 * maintainer is Zerom, Will Son) */
/* This code has been integrated with "Sync Read and Sync Write Example" (whose autor is Ryu Woon Jung (Leon)) */
/* The original source code HAVE BEEN MODIFIED */

#include "motor.hpp"

extern "C"
{
    #include "dynamixel_sdk.h"
}

int Motor::port_num = 0;
uint8_t Motor::dxl_addparam_result = false;  
uint8_t Motor::dxl_getdata_result = false;
int Motor::groupwrite_num;
int Motor::groupread_num;
int Motor::motor_count;
int32_t Motor::dxl_present_position[MOTOR_NUM] = {0};
int Motor::dxl_comm_result;

// Default constructor
Motor::Motor()
{
}

// Helper function
int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}


// This function initializes the motors (opening the port, setting the port baudrate and enabling Dynamixel torque).
// If the motor connection has been successful, the function returns the value 1, otherwise it returns the value 0.
int Motor::InitializeMotor(int motor_id)
{
    // Initialize PortHandler Structs
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    if (motor_id == 0)
    {
        portHandler(DEVICENAME);
        port_num = 0;
    }

    // Communication result
    dxl_comm_result = COMM_TX_FAIL;

    // Initialise PacketHandler Structs
    packetHandler();
    
    if (motor_id == 0)
    {
        // Initialise Groupsyncwrite Structs
        groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
    
        motor_count = 0;
        // Open port
        if (openPort(port_num))
        {
            printf("MOTOR - Succeeded to open the port!\n");
        }
        else
        {
            printf("MOTOR - Failed to open the port!\n");
            return 0;
        }

        // Set port baudrate
        if (setBaudRate(port_num, BAUDRATE))
        {
            printf("MOTOR - Succeeded to change the baudrate!\n");
        }
        else
        {
            printf("MOTOR - Failed to change the baudrate!\n");
            return 0;
        }
    }
    
    // Counting the number of correctly opened motors
    motor_count = motor_count + 1;
    dxl_present_position[motor_id] = 0;
    

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
    else
    {
        std::cout << "MOTOR - Dynamixel with ID " << motor_id << " has been successfully connected" << std::endl;
    }
}


// This function closes the motors (disabling Dynamixel torque, and closing the port)
void Motor::TerminateMotor(int motor_id, int last)
{
    // Disable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }

    // Close port
    if (last)
    {
        closePort(port_num);
    }
}


// This function moves the motors so that they reache a goal position passed as input
// Note that bidirectional communication has been removed to make the movement of the motors faster
void Motor::MoveAllMotors(std::vector<int> goal_position)
{
    int cached_goal_position[MOTOR_NUM] = {0};
    int stop = 0;
    
    for (int k = 0; k < MOTOR_NUM; k++)
    {
        cached_goal_position[k] = goal_position[k];
    }
    
	for (int i = 0; i < motor_count; i++)
	{
		dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, i, cached_goal_position[i], LEN_PRO_GOAL_POSITION);
    	if (dxl_addparam_result != True)
    	{
      		std::cout << "MOTOR - groupSyncWrite addparam failed (motor with ID " << i << ")" << std::endl;
      		return;
    	}
	}
	
	// Syncwrite goal position
    /// ADD THE NEXT LINE AGAIN TO MOVE MOTORS
    groupSyncWriteTxPacket(groupwrite_num);
    //if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    //{
    //	printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    //}

    // Clear syncwrite parameter storage
    groupSyncWriteClearParam(groupwrite_num);

}    
