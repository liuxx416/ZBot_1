#ifndef ENVIROBOT_H
#define ENVIROBOT_H

#include "eye.hpp"
#include "motor.hpp"
#include "laterPT.hpp"
#include "behaviourDeterminator.hpp"
#include "cpg.hpp"
#include <thread>
#include <chrono>
#include "cscore.h"
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <fstream>
#include "ina219.h"

// The following line was used to collect data while performing experiments on the robot prototype
// #include <fstream>

#define CONTROL_STEP 60 // In milliseconds

#define weigth_LPF_mPT 0.05
//The following omega and bias are moved to private
//double omega_DSs = 0.08;
//double omega_DSs2 = 0.08; // 0.095
//double bias_DSs = 110;
//double bias_DSs2 = 110; //85
#define omega_PT05 1
#define bias_PT05 2
#define omega_PT1 1.5
#define bias_PT1 1.2

#define omega_LHB 4
#define bias_LHB 0.6
#define omega_VSPNs 3
#define bias_VSPNs 0.7 
#define noise_to_SPNs 0.1

#define MOTOR_NUM 6 // Number of motors

// Define the time (in milliseconds) for which the program should run.
//#define RUN_TIME 120e3 (Currently passed as command line argument by the GUI)



class Envirobot {

private:

/// Log files for the experiments
/// Direction Selective Cells in Pretectum
//std::ofstream log_fileDSGCR0;
//std::ofstream log_fileDSGCR1;
//std::ofstream log_fileDSGCR2;
//std::ofstream log_fileDSGCR3;
//std::ofstream log_fileDSGCL0;
//std::ofstream log_fileDSGCL1;
//std::ofstream log_fileDSGCL2;
//std::ofstream log_fileDSGCL3;
/// Direction Selective Cells in late Pretectum
//std::ofstream log_fileLB;
//std::ofstream log_fileLIB;
//std::ofstream log_fileLIOB;
//std::ofstream log_fileLMM;
//std::ofstream log_fileLIMM;
//std::ofstream log_fileLOML;
//std::ofstream log_fileLS;
//std::ofstream log_fileLOB;
//std::ofstream log_fileRB;
//std::ofstream log_fileRIB;
//std::ofstream log_fileRIOB;
//std::ofstream log_fileRMM;
//std::ofstream log_fileRIMM;
//std::ofstream log_fileROML;
//std::ofstream log_fileRS;
//std::ofstream log_fileROB;
/// LMLF, RMLF, LLHB, RLHB
//std::ofstream log_fileLMLF;
//std::ofstream log_fileRMLF;
//std::ofstream log_fileLLHB;
//std::ofstream log_fileRLHB;
/// All the neurons
FILE * log_file;
FILE * log_file_MOTORS;
FILE * log_file_PowerMeter;
FILE * log_file_TailBeatingAmplitude;

double omega_DSs = 0.08;
double omega_DSs2 = 0.08; // 0.095
double bias_DSs = 110;
double bias_DSs2 = 110; //85

  /***************************************************************************
  *  @brief: A vector holding eye objects
  ***************************************************************************/
  std::vector<Eye> d_eyes;
  
  /***************************************************************************
  *  @brief: A vector holding motor objects
  ***************************************************************************/
  std::vector<Motor> d_mot;

  /***************************************************************************
  *  @brief: LaterPT object
  ***************************************************************************/
  LaterPT ltPT;

  /***************************************************************************
  *  @brief: CPG object
  ***************************************************************************/
  CPG motor_CPG;

  /***************************************************************************
  *  @brief: BehaviourDeterminator object
  ***************************************************************************/
  BehaviourDeterminator behaviour_det;

  /***************************************************************************
  *  @brief: An AD (analog-digital) object
  ***************************************************************************/   
  AD ADinputs;

  /***************************************************************************
  *  @brief: An AD object
  ***************************************************************************/   
  INA219 PowerMeter;

  /***************************************************************************
  *  @brief: Value assumed by the Direction Selective cell in the right
  *          pretectum (of type 0) after being smoothed using a Low-Pass
  *          Filter.
  ***************************************************************************/
  double smoothed_outDSGC_L_0 = 0;

  /***************************************************************************
  *  @brief: Value assumed by the Direction Selective cell in the right
  *          pretectum (of type 1) after being smoothed using a Low-Pass
  *          Filter.
  ***************************************************************************/
  double smoothed_outDSGC_L_1 = 0;

  /***************************************************************************
  *  @brief: Value assumed by the Direction Selective cell in the right
  *          pretectum (of type 2) after being smoothed using a Low-Pass
  *          Filter.
  ***************************************************************************/
  double smoothed_outDSGC_L_2 = 0;

  /***************************************************************************
  *  @brief: Value assumed by the Direction Selective cell in the right
  *          pretectum (of type 3) after being smoothed using a Low-Pass
  *          Filter.
  ***************************************************************************/
  double smoothed_outDSGC_L_3 = 0;

  /***************************************************************************
  *  @brief: Value assumed by the Direction Selective cell in the left
  *          pretectum (of type 0) after being smoothed using a Low-Pass
  *          Filter.
  ***************************************************************************/
  double smoothed_outDSGC_R_0 = 0;

  /***************************************************************************
  *  @brief: Value assumed by the Direction Selective cell in the left
  *          pretectum (of type 1) after being smoothed using a Low-Pass
  *          Filter.
  ***************************************************************************/
  double smoothed_outDSGC_R_1 = 0;

  /***************************************************************************
  *  @brief: Value assumed by the Direction Selective cell in the left
  *          pretectum (of type 2) after being smoothed using a Low-Pass
  *          Filter.
  ***************************************************************************/
  double smoothed_outDSGC_R_2 = 0;

  /***************************************************************************
  *  @brief: Value assumed by the Direction Selective cell in the left
  *          pretectum (of type 3) after being smoothed using a Low-Pass
  *          Filter.
  ***************************************************************************/
  double smoothed_outDSGC_R_3 = 0;
  
  /***************************************************************************
  *  @brief: Value assumed by the Left nMLF after being smoothed using a
  *          Low-Pass Filter.
 ***************************************************************************/
  double LMLF = 0;

  /***************************************************************************
  *  @brief: Value assumed by the Right nMLF after being smoothed using a
  *          Low-Pass Filter.
  ***************************************************************************/
  double RMLF = 0;

  /***************************************************************************
  *  @brief: Value assumed by the Left LHB after being smoothed using a 
  *          Low-Pass Filter.
  ***************************************************************************/
  double LLHB = 0;

  /***************************************************************************
  *  @brief: Value assumed by the Right LHB after being smoothed using a 
  *          Low-Pass Filter.
  ***************************************************************************/
  double RLHB = 0;

  /***************************************************************************
  *  @brief: Tail phase.
  ***************************************************************************/
  double phase = 0;

  /***************************************************************************
  *  @brief: Smoothed value of the current_swim, influenced by the value
  *          assumed by SS_MLF_countdown.
  ***************************************************************************/
  double swim_amplitude = 0;

  /***************************************************************************
  *  @brief: Smoothed value of the left descending amplitude, influenced by
  *          the value assumed by SS_MLF_countdown.
  ***************************************************************************/
  double left_swim_amplitude = 0;

  /***************************************************************************
  *  @brief: Smoothed value of the right descending amplitude, influenced by
  *          the value assumed by SS_MLF_countdown.
  ***************************************************************************/
  double right_swim_amplitude = 0;

  /***************************************************************************
  *  @brief: Smoothed value of current_LVSPNs (value assumed by the
  *          ventromedial Spinal Projection Neurons - vSPNs).
  ***************************************************************************/
  double LVSPNs_output = 0;

  /***************************************************************************
  *  @brief: Smoothed value of current_RVSPNs (value assumed by the
  *          ventromedial Spinal Projection Neurons - vSPNs).
  ***************************************************************************/
  double RVSPNs_output = 0;

  /***************************************************************************
  *  @brief: Activation variable for the Shishi-odoshi.
  ***************************************************************************/
  double SS_MLF = 0;

  /***************************************************************************
  *  @brief: Countdown variable for the Shishi-odoshi.
  ***************************************************************************/
  double SS_MLF_countdown = 0;
  
  /***************************************************************************
  *  @brief: Countdown variable for the Left VSPN.
  ***************************************************************************/
  double SS_LVSPNs_countdown = 0;
  
  /***************************************************************************
  *  @brief: Countdown variable for the Right VSPN.
  ***************************************************************************/
  double SS_RVSPNs_countdown = 0;

  /***************************************************************************
  *  @brief: Action chosen by the robot. If 'command' assumes value -1, the
  *          robot decided to turn left; if 'command' assumes value 0, the
  *          robot decided to swim forward; if 'command' assumes value 1,
  *          the robot decided to turn right.
  ***************************************************************************/
  int command;

  /***************************************************************************
  *  @brief: Flag that indicates if the robot is doing a bout
  ***************************************************************************/
  bool bout_flag = 0; // 0 = no bout | 1 = bout

  /***************************************************************************
  *  @brief: Flag that indicates the direction in which the first tail segment
  *          bends. If 'first_bend_flag' assumes value 0, the first tail
  *          segment bends to the left; if 'first_bend_flag' assumes value 1,
  *          the first tail segment bends to the right.
  ***************************************************************************/
  bool first_bend_flag = 0;

  /***************************************************************************
  *  @brief: Noise rate, responsible of the reduction of the countdown
  *          variable for the Shishi-odoshi 'SS_MLF_countdown'.
  ***************************************************************************/
  double noise_SS_MLF_deduction_rate = 0;

  /***************************************************************************
  *  @brief: Random gaussian noise taken into account to compute the value of
  *          the Left ventromedial Spinal Projection Neuron (vSPN).
  ***************************************************************************/
  double noise_turning_LSPN = 0;

  /***************************************************************************
  *  @brief: Random gaussian noise taken into account to compute the value of
  *          the Right ventromedial Spinal Projection Neuron (vSPN).
  ***************************************************************************/
  double noise_turning_RSPN = 0;

  /***************************************************************************
  *  @brief: Random gaussian noise taken into account to compute the value of
  *          the Left ventromedial Spinal Projection Neuron (vSPN).
  ***************************************************************************/
  double noise_forward_LSPN = 0;

  /***************************************************************************
  *  @brief: Random gaussian noise taken into account to compute the value of
  *          the Right ventromedial Spinal Projection Neuron (vSPN).
  ***************************************************************************/
  double noise_forward_RSPN = 0;

  /***************************************************************************
  *  @brief: Target motor positions (in radians).
  ***************************************************************************/
  std::vector<double> target_motor_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /***************************************************************************
  *  @brief: Goal motor positions, values readable directly by the motors.
  ***************************************************************************/
  std::vector<int> goal_motor_position = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  /***************************************************************************
  *  @brief: X coordinate of the tail position.
  ***************************************************************************/
  double tail_position_x;

  /***************************************************************************
  *  @brief: Y coordinate of the tail position.
  ***************************************************************************/
  double tail_position_y;

  /***************************************************************************
  *  @brief: Tail angle.
  ***************************************************************************/
  double tail_angle;

  /***************************************************************************
  *  @brief: Goal position in degrees for a single motor.
  ***************************************************************************/
  double goal_position_deg;

  /***************************************************************************
  *  @brief: Flag that indicates wheter the motors have been successfully
  *          connected. If the flag has value 1, all the motors are connected.
  ***************************************************************************/
  int motors_connected;

  /***************************************************************************
  *  @brief: Values of AD inputs
  ***************************************************************************/
  int ADInputValues[4] = {0, 0, 0, 0};
  
  /***************************************************************************
  *  @brief: Values of AD inputs
  ***************************************************************************/
  float PowerMeterOutput[3] = {0,0,0};
public:

  /***************************************************************************
  *  @brief:   Default constructor.
  *  @warning: This constructor works if only ONE camera is connected to the
  *            hardware. Otherwise, use the custom constructor.
  ***************************************************************************/
  Envirobot();

  /***************************************************************************
  *  @brief: Custom constructor.
  ***************************************************************************/
  Envirobot(std::vector<std::pair<Orientation, std::string>> cameraInfo);

  /***************************************************************************
  *  @brief: Destructor.
  ***************************************************************************/
  ~Envirobot();

  /***************************************************************************
  *  @brief: Initialize robot's cameras.
  *
  *  @details: Method to initialize all eyes of the robot and
  *            begin image acquisition.
  ***************************************************************************/
  void OpenEyes(int GUI_FLAG);
  
  /***************************************************************************
  *  @brief: Initialize robot's motors.
  *
  *  @details: This function initializes all motors of the robot. If the
  *            motors connection has been successful, the function returns
  *            the value 1, otherwise it returns the value 0.
  ***************************************************************************/
  int OpenMotors();

  /***************************************************************************
  *  @brief: Shutdown robot's cameras.
  *
  *  @details: Method to close all eyes of the robot and
  *            stop image acquisition.
  ***************************************************************************/
  void CloseEyes();
  
  /***************************************************************************
  *  @brief: Terminate robot's motors.
  *
  *  @details: This function closes all the motors of the robot.
  ***************************************************************************/
  void CloseMotors();

  /***************************************************************************
  *  @brief: Updates the visual information captured by a single eye.
  *
  *  @details: This function updates the visual information for the eye passed
  *            as input. From this eye, a new frame or three new frames
  *            (depending on the acquisition mode) are acquired.
  ***************************************************************************/
  void UpdateVisualInformation(Eye* eye, int startup, int ex_time_s, int GUI_FLAG, std::string IPv4_to_contact);

  /***************************************************************************
  *  @brief: Initializes the robot.
  *
  *  @details: Method to initialize the robot (it opens the robot's eyes,
  *            initializes the robot's motors, and gives the visual network
  *            time to stabilize).
  ***************************************************************************/
  void Initialize(int GUI_FLAG);

  /***************************************************************************
  *  @brief: Implements the OptoMotor Response of the fish.
  *
  *  @details: This function implements the OptoMotor Response of the fish,
  *            calculating the Direction Selective Ganglion Cells in the
  *            retina, the Direction Selective cells in the pretectum,
  *            the left nMLF value, the right nMLF value, the left LHB value,
  *            the right LHB value.
  ***************************************************************************/
  void ExecuteOMR_and_GetTargetMotorPosition(int startup, int ex_time_s, int GUI_FLAG, std::string IPv4_to_contact);
  
    /***************************************************************************
  *  @brief: Handles the communication with the Graphic User Interface (GUI).
  *
  *  @details: This function handles the communication with the GUI. In particular,
  *            it transmits the live stream from the cameras, and the live
  *            values of all the neurons related to OMR. It also checks whether
  *            the GUI gave the command of stopping the execution of the program.
  ***************************************************************************/
  void GUICommunication(int ex_time_s, std::string IPv4_to_contact);
  
  /***************************************************************************
  *  @brief: Moves the robot's motors and computes the motors' target position.
  *
  *  @details: This function moves all the motors of the robot after computing
  *            the target position of each of the motors.
  ***************************************************************************/
  void MoveMotors(int startup, int ex_time_s);

  /***************************************************************************
  *  @brief: A thread contains assistance function.
  *
  *  @details: This thread contains refreshing AD inputs (currently)
  ***************************************************************************/  
  void AssistanceThread(int startup, int ex_time_s, int GUI_FLAG, std::string IPv4_to_contact);

  /***************************************************************************
  *  @brief: A thread to measure the power consupmtion.
  *
  *  @details: This thread contains refreshing inputs from the powermeter
  ***************************************************************************/  
  void PowerMeterThread(int startup, int ex_time_s, int GUI_FLAG, std::string IPv4_to_contact);
  
  /***************************************************************************
  *  @brief: Computes the tail position and angle of the robot.
  *
  *  @details: This function computes the tail position and the tail angle
  *            (the tail angle is calculated by forward kinematics).
  ***************************************************************************/
  void UpdateCoordinates();
  
  /***************************************************************************
  *  @brief: Converts the motors target positions.
  *
  *  @details: This function converts the motors positions (in radians)
  *            to a value suitable for the motors currently in use.
  ***************************************************************************/
  std::vector<int> ConvertMotorPosition(std::vector<double> INPosition);


  /***************************************************************************
  *  @brief: Carries out a full cycle of operations
  *
  *  @details: Method to go through all steps of the controller's
  *            processing cycle. In particular, the visual information for
  *            each of the eyes is updated, the values of the neurons involved
  *            in the OMR are updated, the target motors positions are updated
  *            and the motors are moved.
  ***************************************************************************/
  void Cycle(int ex_time_s, int GUI_FLAG, std::string IPv4_to_contact);

  /***************************************************************************
  *  @brief: Stops the robot
  *
  *  @details: Method to stop the robot.
  *             1) Closes the eyes
                2) Closes the motors
  ***************************************************************************/
  void Stop();

  /***************************************************************************
  *  @brief: Gets the current camera frames and merges them into a single image.
  *
  *  @details: Method to get the current camera frames for each camera, and
  *            merge them into a single frame.
  ***************************************************************************/
  cv::Mat MergeFrames(int whichFrame);
  
};

#endif
