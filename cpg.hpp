#ifndef CPG_H
#define CPG_H

#include <iostream>
#include <vector>
#include <math.h>

#define E_N 2.71828
#define bias_secondary_motor_neuron 2
#define omega_secondary_motor_neuron 1.4

class CPG {

    private:

    /***************************************************************************
    *  @brief: Vector indicating the ampitude of each of the segments composing
    *          the robot (comulative amplitude).
    ***************************************************************************/
    const double seg_amplitude[7] = {0.202952981, 0.36415025, 0.481232303, 0.593806825, 0.69596167, 1};

    /***************************************************************************
    *  @brief: Value assumed by the left motor neuron currently under analysis
    *          before being passed through a sigmoid function.
    ***************************************************************************/
    double left_motor_neuron_input;

    /***************************************************************************
    *  @brief: Value assumed by the right motor neuron currently under analysis
    *          before being passed through a sigmoid function.
    ***************************************************************************/
    double right_motor_neuron_input;

    /***************************************************************************
    *  @brief: Values for the left Central Pattern Generator (CPG) for each of
    *          the segments composing the robot.
    ***************************************************************************/
    double left_CPG[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    /***************************************************************************
    *  @brief: Values for the right Central Pattern Generator (CPG) for each of
    *          the segments composing the robot.
    ***************************************************************************/
    double right_CPG[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    /***************************************************************************
    *  @brief: Values assumed by the left motor neuron for the various segments
    *          composing the robot after being passed through a sigmoid function.
    ***************************************************************************/
    double left_secondary_motor_neuron[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    /***************************************************************************
    *  @brief: Values assumed by the right motor neuron for the various segments
    *          composing the robot after being passed through a sigmoid function.
    ***************************************************************************/
    double right_secondary_motor_neuron[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    /***************************************************************************
    *  @brief: Target motor positions (in radians) for all the motors present
    *          on the robot.
    ***************************************************************************/
    std::vector<double> target_motor_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    public:

    /***************************************************************************
    *  @brief: Default constructor.
    ***************************************************************************/
    CPG();

    /***************************************************************************
    *  @brief: Computes of the target motor positions for all the motors.
    *
    *  @details: This function computes the target motor positions for all the
    *            motors that are present on the robot. First of all, Central
    *            Pattern Generators (CPGs) are employed to generate rythmic
    *            swimming (also taking into account whether the first tail
    *            segment is bending to the left or to the right). The values
    *            of the left and right CPGs are then combined with the values
    *            assumed by the ventromedial Spinal Projection Neurons (vSPNs)
    *            that are passed as inputs. The obtained values are then used
    *            to compute the target motors positions (in radians).
    ***************************************************************************/
    void UpdateCPG(double left_amplitude, double right_amplitude, double LMLF, double RMLF, double LVSPNs_output, double RVSPNs_output, bool bout_flag, bool first_bend_flag, double phase);
 
    /***************************************************************************
    *  @brief: Returns a vector containing the target motor positions for all
    *          motors.
    *
    *  @details: This function returns a vector of doubles that contain the
    *            target motor position (in radians) for all the motors on the
    *            robot.
    ***************************************************************************/
    std::vector<double> GetMotorPosition();

};

#endif
