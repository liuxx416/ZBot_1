

#ifndef EYE_H
#define EYE_H

#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "xiApiPlusOcv.hpp"
#include "parameters.h"
#include <string>
#include <thread>


// Camera Parameters
#define FRAME_WIDTH 368
#define FRAME_HEIGHT 276

// The Region Of Interest is equal to the whole visual field (reduced to increase speed)
#define ROI_WIDTH  184
#define ROI_HEIGHT 80
#define OFFSET_X 0
#define OFFSET_Y_LEFT  0
#define OFFSET_Y_RIGHT 0

#define E_N 2.71828

// Parameters used in the sigmoid function of the OFF Bipolar Cells
//float omega_OFF_BC = 0.4;
//float bias_OFF_BC = 10.0;
// Parameters used in the sigmoid function of the OFF Direction Selective ganglion Cells
//float omega_OFF_DSC = 10.0;
//float bias_OFF_DSC = 0.6;

#define GAP 2
#define weight_iBCs 2
#define GAPbi 2

// Two acquisitions mode are possible:
// ACQUISITION_MODE = 1 -> all the three frames are acquired before proceding
// ACQUISITION_MODE = 0 -> one frame is acquired at each cycle
// Note: by default, the acquisition mode that should be choosen is 0, as it is more similar to what
// actually happens in biological fishes and as it reduces significantly the computational time of the overall cycle.
// The acquisition mode should be set to 1 only if the time that elapses between the acquisition of two successive frames
// is too high (higher than 60 - 70 ms)
#define ACQUISITION_MODE 1

// Note: this variable is used only if the acquisition mode is 1
// Defines the time interval (in milliseconds) that must pass between the acquisition of two successive samples
#define acquisition_interval 20

/***************************************************************************
*  @brief: An enumerated type to specify eye orientation
*
*  @details: An eye can be either on the right side of the robot, or on the left
*            side of the robot.
***************************************************************************/
enum Orientation {
  Left,
  Right
};


class Eye{


private:
// Parameters used in the sigmoid function of the OFF Bipolar Cells
float omega_OFF_BC = 0.5;
float bias_OFF_BC = 10.0;
// Parameters used in the sigmoid function of the OFF Direction Selective ganglion Cells
float omega_OFF_DSC = 10.0;
float bias_OFF_DSC = 0.6;

float filtedBCAverage = 0;
  /***************************************************************************
  *  @brief: A bool variable that indicates whether we are in startup phase.
  ***************************************************************************/
  bool start = 1;

  /***************************************************************************
  *  @brief: time_point that indicates the time that elapsed between the
             acquisition of the last two frames for the left eye.
  ***************************************************************************/
  std::chrono::high_resolution_clock::time_point globalTimeLeft;
  /***************************************************************************
  *  @brief: time_point that indicates the time that elapsed between the
             acquisition of the last two frames for the right eye.
  ***************************************************************************/
  std::chrono::high_resolution_clock::time_point globalTimeRight;
  /***************************************************************************
  *  @brief: time_point that indicates the time that the program automatically adjust the camera parameters.
  ***************************************************************************/
  std::chrono::high_resolution_clock::time_point lastAdjustTime = std::chrono::high_resolution_clock::now();

  //pre-calculated sigmoid function for bipolar cells (pre-calcuated since there is a huge number of bipolar cells)
  //Omega = 0.5, bias = 20
  const float Sigmoid_BC[256] = {0.000045397868702, 0.000074846227511, 0.000123394575986, 0.000203426978055, 0.000335350130466, 0.000552778636924, 0.000911051194401, 0.001501182256737, 0.002472623156635, 0.004070137715896, 0.006692850924285, 0.010986942630593, 0.017986209962092, 0.029312230751356, 0.047425873177567, 0.075858180021244, 0.119202922022118, 0.182425523806356, 0.268941421369995, 0.377540668798145, 0.500000000000000, 0.622459331201855, 0.731058578630005, 0.817574476193644, 0.880797077977882, 0.924141819978757, 0.952574126822433, 0.970687769248644, 0.982013790037908, 0.989013057369407, 0.993307149075715, 0.995929862284104, 0.997527376843365, 0.998498817743263, 0.999088948805599, 0.999447221363076, 0.999664649869534, 0.999796573021945, 0.999876605424014, 0.999925153772489, 0.999954602131298, 0.999972464308885, 0.999983298578152, 0.999989870009019, 0.999993855825398, 0.999996273360716, 0.999997739675702, 0.999998629042793, 0.999999168471972, 0.999999495652592, 0.999999694097773, 0.999999814460898, 0.999999887464838, 0.999999931743971, 0.999999958600624, 0.999999974890009, 0.999999984770021, 0.999999990762550, 0.999999994397204, 0.999999996601732, 0.999999997938846, 0.999999998749847, 0.999999999241744, 0.999999999540095, 0.999999999721053, 0.999999999830810, 0.999999999897381, 0.999999999937758, 0.999999999962249, 0.999999999977103, 0.999999999986112, 0.999999999991577, 0.999999999994891, 0.999999999996901, 0.999999999998120, 0.999999999998860, 0.999999999999309, 0.999999999999581, 0.999999999999746, 0.999999999999846, 0.999999999999907, 0.999999999999943, 0.999999999999966, 0.999999999999979, 0.999999999999987, 0.999999999999992, 0.999999999999995, 0.999999999999997, 0.999999999999998, 0.999999999999999, 0.999999999999999, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1}; 
  //Omega = 0.2, bias = 20
  const float Sigmoid_BC2[256] = {0.017986257 , 0.021881326 , 0.026597056 , 0.032295536 , 0.039165804 , 0.047425964 , 0.057324278 , 0.069138533 , 0.08317282 , 0.099750622 , 0.119203063 , 0.141851212 , 0.167981765 , 0.197816261 , 0.23147536 , 0.268941554 , 0.310025634 , 0.354343786 , 0.401312405 , 0.450166036 , 0.5 , 0.549833964 , 0.598687595 , 0.645656214 , 0.689974366 , 0.731058446 , 0.76852464 , 0.802183739 , 0.832018235 , 0.858148788 , 0.880796937 , 0.900249378 , 0.91682718 , 0.930861467 , 0.942675722 , 0.952574036 , 0.960834196 , 0.967704464 , 0.973402944 , 0.978118674 , 0.982013743 , 0.985225927 , 0.98787153 , 0.990048168 , 0.991837403 , 0.993307127 , 0.994513682 , 0.995503711 , 0.996315746 , 0.996981572 , 0.997527367 , 0.997974671 , 0.998341192 , 0.998641474 , 0.998887459 , 0.999088945 , 0.999253968 , 0.999389118 , 0.999499796 , 0.999590431 , 0.999664648 , 0.99972542 , 0.999775182 , 0.999815927 , 0.999849289 , 0.999876605 , 0.99989897 , 0.999917282 , 0.999932275 , 0.999944551 , 0.999954602 , 0.999962831 , 0.999969568 , 0.999975084 , 0.999979601 , 0.999983298 , 0.999986326 , 0.999988805 , 0.999990834 , 0.999992495 , 0.999993856 , 0.99999497 , 0.999995881 , 0.999996628 , 0.999997239 , 0.99999774 , 0.999998149 , 0.999998485 , 0.999998759 , 0.999998984 , 0.999999168 , 0.999999319 , 0.999999443 , 0.999999544 , 0.999999626 , 0.999999694 , 0.99999975 , 0.999999795 , 0.999999832 , 0.999999863 , 0.999999887 , 0.999999908 , 0.999999925 , 0.999999938 , 0.999999949 , 0.999999959 , 0.999999966 , 0.999999972 , 0.999999977 , 0.999999981 , 0.999999985 , 0.999999988 , 0.99999999 , 0.999999992 , 0.999999993 , 0.999999994 , 0.999999995 , 0.999999996 , 0.999999997 , 0.999999997 , 0.999999998 , 0.999999998 , 0.999999999 , 0.999999999 , 0.999999999 , 0.999999999 , 0.999999999 , 0.999999999 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1};
  //Omega = 0.5, bias = 10
  const float Sigmoid_BC3[256] = {0.006692873 , 0.010986976 , 0.017986257 , 0.029312298 , 0.047425964 , 0.075858298 , 0.119203063 , 0.182425674 , 0.268941554 , 0.377540748 , 0.5 , 0.622459252 , 0.731058446 , 0.817574326 , 0.880796937 , 0.924141702 , 0.952574036 , 0.970687702 , 0.982013743 , 0.989013024 , 0.993307127 , 0.995929847 , 0.997527367 , 0.998498811 , 0.999088945 , 0.999447219 , 0.999664648 , 0.999796572 , 0.999876605 , 0.999925153 , 0.999954602 , 0.999972464 , 0.999983298 , 0.99998987 , 0.999993856 , 0.999996273 , 0.99999774 , 0.999998629 , 0.999999168 , 0.999999496 , 0.999999694 , 0.999999814 , 0.999999887 , 0.999999932 , 0.999999959 , 0.999999975 , 0.999999985 , 0.999999991 , 0.999999994 , 0.999999997 , 0.999999998 , 0.999999999 , 0.999999999 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1};
  //Omega = 0.2, bias = 10
  const float Sigmoid_BC4[256] = {0.119203063 , 0.141851212 , 0.167981765 , 0.197816261 , 0.23147536 , 0.268941554 , 0.310025634 , 0.354343786 , 0.401312405 , 0.450166036 , 0.5 , 0.549833964 , 0.598687595 , 0.645656214 , 0.689974366 , 0.731058446 , 0.76852464 , 0.802183739 , 0.832018235 , 0.858148788 , 0.880796937 , 0.900249378 , 0.91682718 , 0.930861467 , 0.942675722 , 0.952574036 , 0.960834196 , 0.967704464 , 0.973402944 , 0.978118674 , 0.982013743 , 0.985225927 , 0.98787153 , 0.990048168 , 0.991837403 , 0.993307127 , 0.994513682 , 0.995503711 , 0.996315746 , 0.996981572 , 0.997527367 , 0.997974671 , 0.998341192 , 0.998641474 , 0.998887459 , 0.999088945 , 0.999253968 , 0.999389118 , 0.999499796 , 0.999590431 , 0.999664648 , 0.99972542 , 0.999775182 , 0.999815927 , 0.999849289 , 0.999876605 , 0.99989897 , 0.999917282 , 0.999932275 , 0.999944551 , 0.999954602 , 0.999962831 , 0.999969568 , 0.999975084 , 0.999979601 , 0.999983298 , 0.999986326 , 0.999988805 , 0.999990834 , 0.999992495 , 0.999993856 , 0.99999497 , 0.999995881 , 0.999996628 , 0.999997239 , 0.99999774 , 0.999998149 , 0.999998485 , 0.999998759 , 0.999998984 , 0.999999168 , 0.999999319 , 0.999999443 , 0.999999544 , 0.999999626 , 0.999999694 , 0.99999975 , 0.999999795 , 0.999999832 , 0.999999863 , 0.999999887 , 0.999999908 , 0.999999925 , 0.999999938 , 0.999999949 , 0.999999959 , 0.999999966 , 0.999999972 , 0.999999977 , 0.999999981 , 0.999999985 , 0.999999988 , 0.99999999 , 0.999999992 , 0.999999993 , 0.999999994 , 0.999999995 , 0.999999996 , 0.999999997 , 0.999999997 , 0.999999998 , 0.999999998 , 0.999999999 , 0.999999999 , 0.999999999 , 0.999999999 , 0.999999999 , 0.999999999 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1};
  //Omega = 1, bias = 6
  const float Sigmoid_BC5[256] = {0.002472633 , 0.006692873 , 0.017986257 , 0.047425964 , 0.119203063 , 0.268941554 , 0.5 , 0.731058446 , 0.880796937 , 0.952574036 , 0.982013743 , 0.993307127 , 0.997527367 , 0.999088945 , 0.999664648 , 0.999876605 , 0.999954602 , 0.999983298 , 0.999993856 , 0.99999774 , 0.999999168 , 0.999999694 , 0.999999887 , 0.999999959 , 0.999999985 , 0.999999994 , 0.999999998 , 0.999999999 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,};
 

  /***************************************************************************
  *  @brief: A camera object.
  ***************************************************************************/
  xiAPIplusCameraOcv cam;

  /***************************************************************************
  *  @brief: A string holding the camera's serial number
  *
  *  @details: For the system currently in use, the serial numbers are:
  *             - Camera 1: 25795659
  *             - Camera 2: 25794959
  ***************************************************************************/
  std::string CAMERA_SERIAL_NUMBER;

  /***************************************************************************
  *  @brief: Integers representing the eye's field of vision (FOV). This is
  *          specified in terms of the number of vertical and horizontal pixels.
  ***************************************************************************/
  int FOV_HEIGHT;
  int FOV_WIDTH;

  /***************************************************************************
  *  @brief: A constant specifying the eye's orientation.
  ***************************************************************************/
  Orientation EYE_ORIENTATION;

  /***************************************************************************
  *  @brief: Images stored in opencv Mat structures.
             Note that the three most recent frames are stored for each eye.
  ***************************************************************************/
  cv::Mat d_currentProcessedFrame;
  cv::Mat d_currentProcessedFrame_minus2;
  cv::Mat d_currentProcessedFrame_minus1;

  cv::Mat acquired_ProcessedFrame;
  cv::Mat acquired_ProcessedFrame_minus2;
  cv::Mat acquired_ProcessedFrame_minus1;
  
  cv::Mat acquired_ProcessedGUIFrame;
  
    /***************************************************************************
  *  @brief: Processed consecutive frames are stored in matrices of doubles.
  *          Each frame contains, pixel by pixel, the mean of the Red and Green
  *          channels of the corresponding pixel of the frames obtained by the
  *          camera. The processed frames have therefore a single channel.
  ***************************************************************************/
  double frameCurrentRG[ROI_HEIGHT][ROI_WIDTH];
  double frameMinus1RG[ROI_HEIGHT][ROI_WIDTH]; 
  double frameMinus2RG[ROI_HEIGHT][ROI_WIDTH];

  double OFF_BC[FRAME_HEIGHT][FRAME_WIDTH];
  double lastOFF_BC[FRAME_HEIGHT][FRAME_WIDTH];
  double OFF_DSC[FRAME_HEIGHT][FRAME_WIDTH];
  double OFF_DSC_input[FRAME_HEIGHT][FRAME_WIDTH];
	
public:


  /***************************************************************************
  *  @brief: Default constructor
  *
  *  @details: This constructor initializes camera variables and parameters.
  ***************************************************************************/
  Eye();


  /***************************************************************************
  *  @brief: Custom constructor
  *
  *  @details: This constructor initializes camera variables and parameters.
  *            This constructor also allows us to specify the orientations of
               multiple cameras.
  *
  *  @params[in]: Orientation
  *                - A variable specifying the camera's orientation. This can
  *                  either be right or left.
  *
  *               std::string
  *                - The serial number of the current XIMEA camera.
  ***************************************************************************/
  Eye(Orientation s, std::string serialNumber);

  /***************************************************************************
  *  @brief: Initializes a camera
  *
  *  @details: Method to "open" the eye - which is to say, initialize the
  *            camera and start the video stream. This method is where the
  *            camera's parameters are specified (such as exposure time, region
  *            of interest, resolution, etc...).
  ***************************************************************************/
  void OpenEye(int GUI);

  /***************************************************************************
  *  @brief: Captures a new image
  *
  *  @details: Method for the eye to capture a new image. This begins by
  *            updating the contents of the d_currentRawFrame matrix. The image
  *            is then rotated by +/-90 degrees (depending on the camera orientaiton)
  *            and the result is stored in d_currentProcessedFrame
  ***************************************************************************/
  cv::Mat GetNewImage(float *timerFrame_ms);
  
  /***************************************************************************
  *  @brief: In an operating mode, captures a single image. In another operating
  *          mode, captures three new successive images with a fixed elapsed time
  *          between the acquisitions.
  *
  *  @details: Method for the eye to capture one or three new successive frames.
  *            If the ACQUISITION_MODE is 1, the function obtains three successive
  *            frames. The time elapsed between the acquisitions is defined by
  *            'acquisition_interval' (in milliseconds).
  *            If the ACQUISITION_MODE is 0, the function obtains a single new
  *            frame each time is is called.
  *            This function calls one or three times the function GetNewImage to
  *            obtain one or three frames.
  ***************************************************************************/
  void GetNewImages();

  /***************************************************************************
  *  @brief: Closes the camera
  *
  *  @details: Method to "close" the eye - which is to say, stop the video
  *            stream and close the camera.
  ***************************************************************************/
  void CloseEye();

  /***************************************************************************
  *  @brief: Method to return the camera's current frame
  ***************************************************************************/
  cv::Mat CurrentFrame(int WhichFrame);
  
  /***************************************************************************
  *  @brief: This function computes the output signal of the Direction
  *          Selective Ganglion Cell of the type indicated as a parameter.
  *
  *  @retval: double
  *             - Returns the output signal of the DSGC
  *  @params[in]: int typeDSGCs
  *                - An integer that indicates the type of the DSGC currently
  *                  under analysis (the possible types are 0 -> superior,
  *                  1 -> anterior, 2 -> inferior, 3 -> posterior).
  ***************************************************************************/
  double GetInputToPretectum(int typeDSGCs);
  
  // Helper function that computes and returns the time elapsed (in milliseconds) from the time_point previousEventTime
  int GetTime(int WhichElapsedTime);

  // Automatically adjust the gain of the camera.
  void AdjustCameraGain();
  // Automatically adjust the transform function of bipolar cells.
  void AdjustTFBC();
  // Automatically adjust the transform function of OFFDSGC.
  void AdjustTFOFFDSGC(float DSGCSum);

  /* Setter functions */
  void SetSerialNumber(std::string SN) { CAMERA_SERIAL_NUMBER = SN; }
  void SetOrientation(Orientation o) { EYE_ORIENTATION = o; }

  /* Getter functions */
  std::string SerialNumber() { return CAMERA_SERIAL_NUMBER; }
  Orientation EyeOrientation() { return EYE_ORIENTATION; }
};

#endif
