#include "laterPT.hpp"

// Helper function to compute the minimum between two float values passed as parameters
float minimum(float fl_1, float fl_2)
{
  if (fl_1 <= fl_2)
  {
    return fl_1;
  }
  return fl_2;
}

// Helper function that returns:
// * a, if a is between b and c
// * b, if a is lower than b
// * c, if a is higher than c
float inbetween(float a, float b, float c)
{
  if (a < b)
  {
    return b;
  }
  if (a > c)
  {
    return c;
  }
  return a;
}

// Default constructor
LaterPT::LaterPT()
{
}

// This function updates the values of the nMLF (both Right nMLF and Left nMLF) starting from the output values of the
// four Direction Selective Ganglion Cells (smoothed and passed through the sigmoid function) that are passed to the function as parameters
void LaterPT::UpdateMLF(double LPT_R_0, double LPT_R_1, double LPT_R_2, double LPT_R_3, double RPT_L_0, double RPT_L_1, double RPT_L_2, double RPT_L_3)
{
    // We compute the values that the multiple types of Direction Selective (DS) cells in the left later PT (Pretectum) acquire. They will be then passed through
    // a sigmoid function, therefore they are labelled as LXXX_input (the first L stands for Left)
    //LoB_input = (RPT_L_0 * weight_LM + LPT_R_2 * weight_MM + LPT_R_3 * 0.5) - (RPT_L_2 * weight_MM * weight_inhibition);
    //LB_input = (RPT_L_0 * weight_LM + LPT_R_2 * weight_MM) - (RPT_L_2 * weight_MM * weight_inhibition + LPT_R_0 * weight_LM * weight_inhibition);
    //LiB_input = (RPT_L_0 * weight_LM + LPT_R_2 * weight_MM) - (LPT_R_0 * weight_LM * weight_inhibition);
    //LioB_input = (RPT_L_0 * weight_LM + LPT_R_2 * weight_MM);
    //LiMm_input = (LPT_R_2 * weight_MM);
    //LMm_input = (LPT_R_2 * weight_MM + LPT_R_1 * 1.0) - (RPT_L_2 * weight_MM * weight_inhibition);
    //LoMl_input = (RPT_L_0 * weight_LM);
    //LS_input = minimum(RPT_L_0 * weight_LM, LPT_R_2 * weight_MM) + (RPT_L_1 * 1.0 + LPT_R_3 * 0.5);
    // Added neurons
    // LEx1_input = LPT_R_3 * 1 + RPT_L_3 * 0.5 - LPT_R_1 * 0.5 - RPT_L_1 * 0.5;
    // LEx2_input = LPT_R_1 *1 + RPT_L_1 * 1 - LPT_R_3 * 1 - RPT_L_3 * 1;

    LoB_input = (RPT_L_0 * weight_LM + LPT_R_2 * weight_MM) - (RPT_L_2 * weight_MM * weight_inhibition) + LPT_R_3 * 1.0 + RPT_L_1 * 0.3;
    LB_input = (RPT_L_0 * weight_LM + LPT_R_2 * weight_MM) - (RPT_L_2 * weight_MM * weight_inhibition + LPT_R_0 * weight_LM * weight_inhibition) + RPT_L_1 * 0.8 + LPT_R_3 * 1.0;
    LiB_input = (RPT_L_0 * weight_LM + LPT_R_2 * weight_MM) - (LPT_R_0 * weight_LM * weight_inhibition) + LPT_R_3 * 0.0;
    LioB_input = (RPT_L_0 * weight_LM + LPT_R_2 * weight_MM);
    LiMm_input = (LPT_R_2 * weight_MM) + LPT_R_1 * 0.8 - RPT_L_3 * 0.8;
    LMm_input = (LPT_R_2 * weight_MM + LPT_R_1 * 1.0) - (RPT_L_2 * weight_MM * weight_inhibition) + RPT_L_1 * 0.5 + LPT_R_3 * 1.0 - RPT_L_3 * 0.5;
    LoMl_input = (RPT_L_0 * weight_LM) + RPT_L_1 * 0.0;
    LS_input = minimum(RPT_L_0 * weight_LM, LPT_R_2 * weight_MM) + (RPT_L_1 * 1.5 + LPT_R_3 * 0.5);

    // We pass the values obtained before through a Sigmoid function, taking into account the appropriate omega and bias
    LoB = 1 / (1 + pow(E_N, -omega_PT2 * (LoB_input - bias_PT2)));
    LB = 1 / (1 + pow(E_N, -omega_PT2 * (LB_input - bias_PT2)));
    LiB = 1 / (1 + pow(E_N, -omega_PT2 * (LiB_input - bias_PT2)));
    LioB = 1 / (1 + pow(E_N, -omega_PT2 * (LioB_input - bias_PT2)));
    LiMm = 1 / (1 + pow(E_N, -omega_PT25 * (LiMm_input - bias_PT25)));
    LMm = 1 / (1 + pow(E_N, -omega_PT25 * (LMm_input - bias_PT25)));
    LoMl = 1 / (1 + pow(E_N, -omega_PT3 * (LoMl_input - bias_PT3)));
    LS = 1 / (1 + pow(E_N, -omega_PT25 * (LS_input - bias_PT25)));

    // We constrain the outputs of the Sigmoid function to be between 0 and 10000 (this are the values of the multiple types of Direction Selective (DS)
    // cells in the left later PT (Pretectum))
    LoB = inbetween(LoB, 0, 10000);
    LB = inbetween(LB, 0, 10000);
    LiB = inbetween(LiB, 0, 10000);
    LioB = inbetween(LioB, 0, 10000);
    LiMm = inbetween(LiMm, 0, 10000);
    LMm = inbetween(LMm, 0, 10000);
    LoMl = inbetween(LoMl, 0, 10000);
    LS = inbetween(LS, 0, 10000);

    // We compute the values that the multiple types of Direction Selective (DS) cells in the right later PT (Pretectum) acquire. They will be then passed through
    // a sigmoid function, therefore they are labelled as RXXX_input (the first R stands for Right)
    //RoB_input = (LPT_R_0 * weight_LM + RPT_L_2 * weight_MM + RPT_L_3 * 0.5) - (LPT_R_2 * weight_MM * weight_inhibition);
    //RB_input = (LPT_R_0 * weight_LM + RPT_L_2 * weight_MM) - (LPT_R_2 * weight_MM * weight_inhibition + RPT_L_0 * weight_LM * weight_inhibition);
    //RiB_input = (LPT_R_0 * weight_LM + RPT_L_2 * weight_MM) - (RPT_L_0 * weight_LM * weight_inhibition);
    //RioB_input = (LPT_R_0 * weight_LM + RPT_L_2 * weight_MM);
    //RiMm_input = (RPT_L_2 * weight_MM);
    //RMm_input = (RPT_L_2 * weight_MM + RPT_L_1 * 1.0) - (LPT_R_2 * weight_MM * weight_inhibition);
    //RoMl_input = (LPT_R_0 * weight_LM);
    //RS_input = minimum(LPT_R_0 * weight_LM, RPT_L_2 * weight_MM) + (LPT_R_1 * 1.0 + RPT_L_3 * 0.5);

        RoB_input = (LPT_R_0 * weight_LM + RPT_L_2 * weight_MM) - (LPT_R_2 * weight_MM * weight_inhibition) + RPT_L_3 * 1.0 + LPT_R_1 * 0.3;
    RB_input = (LPT_R_0 * weight_LM + RPT_L_2 * weight_MM) - (LPT_R_2 * weight_MM * weight_inhibition + RPT_L_0 * weight_LM * weight_inhibition) + LPT_R_1 * 0.8 + RPT_L_3 * 1;
    RiB_input = (LPT_R_0 * weight_LM + RPT_L_2 * weight_MM) - (RPT_L_0 * weight_LM * weight_inhibition) + RPT_L_3 * 0.0;
    RioB_input = (LPT_R_0 * weight_LM + RPT_L_2 * weight_MM);
    RiMm_input = (RPT_L_2 * weight_MM) + RPT_L_1 * 0.8 - LPT_R_3 * 0.8;
    RMm_input = (RPT_L_2 * weight_MM + RPT_L_1 * 1.0) - (LPT_R_2 * weight_MM * weight_inhibition) + LPT_R_1 * 0.5 + RPT_L_3 * 1.0 - LPT_R_3 * 0.5;
    RoMl_input = (LPT_R_0 * weight_LM) + LPT_R_1 * 0.0;
    RS_input = minimum(LPT_R_0 * weight_LM, RPT_L_2 * weight_MM) + (LPT_R_1 * 1.5 + RPT_L_3 * 0.5);

    // We pass the values obtained before through a Sigmoid function, taking into account the appropriate omega and bias
    RoB = 1 / (1 + pow(E_N, -omega_PT2 * (RoB_input - bias_PT2)));
    RB = 1 / (1 + pow(E_N, -omega_PT2 * (RB_input - bias_PT2)));
    RiB = 1 / (1 + pow(E_N, -omega_PT2 * (RiB_input - bias_PT2)));
    RioB = 1 / (1 + pow(E_N, -omega_PT2 * (RioB_input - bias_PT2)));
    RiMm = 1 / (1 + pow(E_N, -omega_PT25 * (RiMm_input - bias_PT25)));
    RMm = 1 / (1 + pow(E_N, -omega_PT25 * (RMm_input - bias_PT25)));
    RoMl = 1 / (1 + pow(E_N, -omega_PT3 * (RoMl_input - bias_PT3)));
    RS_ = 1 / (1 + pow(E_N, -omega_PT25 * (RS_input - bias_PT25)));

    // We constrain the outputs of the Sigmoid function to be between 0 and 10000 (this are the values of the multiple types of Direction Selective (DS)
    // cells in the right later PT (Pretectum))
    RoB = inbetween(RoB, 0, 10000);
    RB = inbetween(RB, 0, 10000);
    RiB = inbetween(RiB, 0, 10000);
    RioB = inbetween(RioB, 0, 10000);
    RiMm = inbetween(RiMm, 0, 10000);
    RMm = inbetween(RMm, 0, 10000);
    RoMl = inbetween(RoMl, 0, 10000);
    RS_ = inbetween(RS_, 0, 10000);

    // We compute the values acquired by the Left nMLF (LMLF) and by the Right nMLF (RMLF) starting from the output values of the
    // Direction Selective (DS) cells in later PT
    PTLMLF_Naumann = 0 - LoB * 0.2 + LB * 0.16 + LiB * 0.25 - LioB * 0.05 + LiMm * 0.6 + LMm * 0.4 - RoMl * 0.8 + LS * 0.2;
    PTRMLF_Naumann = 0 - RoB * 0.2 + RB * 0.16 + RiB * 0.25 - RioB * 0.05 + RiMm * 0.6 + RMm * 0.4 - LoMl * 0.8 + RS_ * 0.2;

    // We pass the values acquired by the Left nMLF (LMLF) and by the Right nMLF (RMLF) (computed before) through a Sigmoid function,
    // taking into account the appropriate omega and bias
    LMLF_Naumann = 1 / (1 + pow(E_N, -omega_LMLF_Naumann * (PTLMLF_Naumann - bias_LMLF_Naumann)));
    RMLF_Naumann = 1 / (1 + pow(E_N, -omega_RMLF_Naumann * (PTRMLF_Naumann - bias_RMLF_Naumann)));
}

// This function updates the values of the LHB (both Right LHB and Left LHB) starting from the output values of the
// Direction Selective (DS) cells in later PT
void LaterPT::UpdateLHB()
{
  // The value of the Left LHB (LLHB) is updated
  PTLaHB = (LoB * 0.575 + LB * 0.533 + LMm * 0.533 + LS * 0.392) - (RoB * 0.333 + RB * 0.333 + RMm * 0.310 + RS_ * 0.080);
  LLHB = 1 / (1 + pow(E_N, -omega_aHB * (PTLaHB - bias_aHB)));

  // The value of the Right LHB (RLHB) is updated
  PTRaHB = (RoB * 0.575 + RB * 0.533 + RMm * 0.533 + RS_ * 0.392) - (LoB * 0.333 + LB * 0.333 + LMm * 0.310 + LS * 0.080);
  RLHB = 1 / (1 + pow(E_N, -omega_aHB * (PTRaHB - bias_aHB)));
}

// This function allows to get the most recent value of nMLF.
// To discriminate whether the Left nMLF value (LMLF) or the Right nMLF value (RMLF) should be returned,
// a direction flag is passed to the function
double LaterPT::GetMLF(int direction_flag)
{
    // direction_flag == 0 -> Left
    // If the direction flag is 0, the Left nMLF value (LMLF) is returned
    // direction_flag == 1 -> Right
    // If the direction_flag is 1, the Right nMLF value (RMLF) is returned

    if (direction_flag == 0)
    {
        return LMLF_Naumann;
    }
    else if (direction_flag == 1)
    {
        return RMLF_Naumann;
    }

    return 0;
}

// This function allows to get the most recent value of LHB.
// To discriminate whether the Left LHB value (LLHB) or the Right LHB (RLHB) value should be returned,
// a direction flag is passed to the function
double LaterPT::GetLHB(int direction_flag)
{
    // direction_flag == 0 -> Left
    // If the direction flag is 0, the Left LHB value (LLHB) is returned
    // direction_flag == 1 -> Right
    // If the direction flag is 1, the Right LHB value (RLHB) is returned

    if (direction_flag == 0)
    {
        return LLHB;
    }
    else if (direction_flag == 1)
    {
        return RLHB;
    }

    return 0;
}

// This function allows to get the most recent value of LOB.
double LaterPT::GetLOB()
{
    return LoB;
}

// This function allows to get the most recent value of LB.
double LaterPT::GetLB()
{
    return LB;
}

// This function allows to get the most recent value of LIB.
double LaterPT::GetLIB()
{
    return LiB;
}

// This function allows to get the most recent value of LIOB.
double LaterPT::GetLIOB()
{
    return LioB;
}

// This function allows to get the most recent value of LIMM.
double LaterPT::GetLIMM()
{
    return LiMm;
}

// This function allows to get the most recent value of LMM.
double LaterPT::GetLMM()
{
    return LMm;
}

// This function allows to get the most recent value of LOML.
double LaterPT::GetLOML()
{
    return LoMl;
}

// This function allows to get the most recent value of LS.
double LaterPT::GetLS()
{
    return LS;
}

// This function allows to get the most recent value of ROB.
double LaterPT::GetROB()
{
    return RoB;
}

// This function allows to get the most recent value of RB.
double LaterPT::GetRB()
{
    return RB;
}

// This function allows to get the most recent value of RIB.
double LaterPT::GetRIB()
{
    return RiB;
}

// This function allows to get the most recent value of RIOB.
double LaterPT::GetRIOB()
{
    return RioB;
}

// This function allows to get the most recent value of RIMM.
double LaterPT::GetRIMM()
{
    return RiMm;
}

// This function allows to get the most recent value of RMM.
double LaterPT::GetRMM()
{
    return RMm;
}

// This function allows to get the most recent value of ROML.
double LaterPT::GetROML()
{
    return RoMl;
}

// This function allows to get the most recent value of RS.
double LaterPT::GetRS()
{
    return RS_;
}
