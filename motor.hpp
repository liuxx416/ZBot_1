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

#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
#include <vector>

#include <stdlib.h>
#include <stdio.h>

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                  // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION            2.0

//#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
//#define DXL2_ID                         2                   // Dynamixel#2 ID: 2

#define BAUDRATE                    57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit of product eManual
#define DXL_MOVING_STATUS_THRESHOLD     5                  // Dynamixel moving status threshold !!TRY TO CHANGE THIS VALUE (e.g. to 5, 10)!!

#define MOTOR_NUM 6


class Motor {

    private:
    
    static uint8_t dxl_addparam_result;  // AddParam result
    static uint8_t dxl_getdata_result;  // GetParam result
    static int motor_count;
    static int groupwrite_num;
    static int groupread_num;

    /***************************************************************************
    *  @brief: Goal position.
    ***************************************************************************/
    int goal_position;

    /***************************************************************************
    *  @brief: Dynamixel error.
    ***************************************************************************/
    uint8_t dxl_error = 0;

    /***************************************************************************
    *  @brief: Current position.
    ***************************************************************************/
    static int32_t dxl_present_position[MOTOR_NUM];  // Read 4 byte Position data

    /***************************************************************************
    *  @brief: Communication result.
    ***************************************************************************/
    static int dxl_comm_result;
    
    /***************************************************************************
    *  @brief: Port number of the Dynamixel motor.
    ***************************************************************************/
    static int port_num;


    public:


    /***************************************************************************
    *  @brief: Default constructor.
    ***************************************************************************/
    Motor();

    /***************************************************************************
    *  @brief: Initializes the motor.
    *
    *  @details: This function initializes the motor by opening the port,
    *            setting the port baudrate and enabling Dynamixel torque.
    *            If the motor connection has been successful, the function
    *            returns the value 1, otherwise it returns the value 0.
    ***************************************************************************/
    int InitializeMotor(int motor_id);

    /***************************************************************************
    *  @brief: Terminates the motor.
    *
    *  @details: This function closes the motor by disabling Dynamixel torque,
    *            and closing the port.
    ***************************************************************************/
    void TerminateMotor(int motor_id, int last);

    /***************************************************************************
    *  @brief: Moves all the motors.
    *
    *  @details: This function moves the motors so that they reache a goal position
    *            passed as input (goal_position).
    ***************************************************************************/
    static void MoveAllMotors(std::vector<int> goal_position);

};

#endif
