#include "cpg.hpp"

// Default constructor
CPG::CPG()
{
}

// This function computes the target motor positions for all the motors of the robot.
// To do so, Central Pattern Generators (CPGs) are employed to generate rythmic swimming (also taking into account whether the
// first tail segment is bending to the left or to the right). The values of the left and right CPGs are then combined with the
// values assumed by the ventromedial Spinal Projection Neurons (vSPNs) that are passed as inputs.
// Finally, the obtained values are used to compute the target motors positions (in radians).
void CPG::UpdateCPG(double left_amplitude, double right_amplitude, double LMLF, double RMLF, double LVSPNs_output, double RVSPNs_output, bool bout_flag, bool first_bend_flag, double phase)
{
    for (int i = 0; i < 6; i++)
    {
        left_motor_neuron_input = 0;
        right_motor_neuron_input = 0;
  
        if (bout_flag == 1)
        {
        if(first_bend_flag == 0) // First tail bend to the left
        {
            left_CPG[i] = sin(phase + i * (1.2 * M_PI / 6)) + 1.01;
            right_CPG[i] = sin(phase + M_PI + i * (1.2 * M_PI / 6)) + 1.01;
        }
        else if (first_bend_flag == 1) // First tail bend to the right
        {
            left_CPG[i] = sin(phase + i * (1.2 * M_PI / 6)) + 1.01;
            right_CPG[i] = sin(phase + M_PI + i * (1.2 * M_PI / 6)) + 1.01;
        }
        else
        { 
            left_CPG[i] = 0.01;
            right_CPG[i] = 0.01;
        }
    }
    else // Not in bout
    {
        left_CPG[i] = 0.01;
        right_CPG[i] = 0.01;
    }
    
    //  The first and second tail segments are also subjected to a bias that depends on the LMLF and RMLF values.
    // The bias on the spinal cord only apply to the 1st and 2nd segment (rostral).
    // See Figure 2 and Figure 4 in [Thiele et al. 2014]
    if (i == 0 || i == 1)
    {
        left_motor_neuron_input = 1.0 * left_amplitude * left_CPG[i] + 1.2 * LMLF + LVSPNs_output * 2;
        right_motor_neuron_input = 1.0 * right_amplitude * right_CPG[i] + 1.2 * RMLF + RVSPNs_output * 2;
    }
    else 
    {
        left_motor_neuron_input = 1.0 * left_amplitude * left_CPG[i] + LVSPNs_output * 2;
        right_motor_neuron_input = 1.0 * right_amplitude * right_CPG[i] + RVSPNs_output * 2;
    }

    // The values obtained are passed through a Sigmoid function
    left_secondary_motor_neuron[i] = 1 / (1 + pow(E_N, -omega_secondary_motor_neuron * (left_motor_neuron_input - bias_secondary_motor_neuron)));
    right_secondary_motor_neuron[i] = 1 / (1 + pow(E_N, -omega_secondary_motor_neuron * (right_motor_neuron_input - bias_secondary_motor_neuron)));

    // The target motor position (in radians) of the i-th motor is computed
    target_motor_position[i] = (right_secondary_motor_neuron[i] - left_secondary_motor_neuron[i]) * 1.5 * seg_amplitude[i];
    target_motor_position[i] = 1.5 * target_motor_position[i];
  }

  // No visual stabilization
  target_motor_position[7] = 0;
  target_motor_position[8] = 0;
}

// This function returns a vector of doubles that contain the target motor position (in radians)
// for all the motors on the robot.
std::vector<double> CPG::GetMotorPosition()
{
    return target_motor_position;
}
