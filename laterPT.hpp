#ifndef LATERPT_H
#define LATERPT_H

#include <iostream>
#include <math.h>

#define E_N 2.71828
#define omega_PT2 2.5
#define bias_PT2 0.8
#define omega_PT25 4
#define bias_PT25 0.6
#define omega_PT3 5
#define bias_PT3 0.4
#define omega_PT35 8
#define bias_PT35 0.3
#define weight_inhibition 1
#define weight_MM 1
#define weight_LM 1
#define omega_LMLF_Naumann 6.5
#define bias_LMLF_Naumann 0.3
#define omega_RMLF_Naumann 6.5
#define bias_RMLF_Naumann 0.3
#define omega_aHB 5
#define bias_aHB 0.65

class LaterPT {

    private:

    /***************************************************************************
    *  @brief: Value acquired by the Left oB Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double LoB_input;

    /***************************************************************************
    *  @brief: Value acquired by the Left B Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double LB_input;

    /***************************************************************************
    *  @brief: Value acquired by the Left iB Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double LiB_input;

    /***************************************************************************
    *  @brief: Value acquired by the Left ioB Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double LioB_input;

    /***************************************************************************
    *  @brief: Value acquired by the Left iMm Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double LiMm_input;

    /***************************************************************************
    *  @brief: Value acquired by the Left Mm Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double LMm_input;

    /***************************************************************************
    *  @brief: Value acquired by the Left oMl Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double LoMl_input;

    /***************************************************************************
    *  @brief: Value acquired by the Left S Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double LS_input;

    /***************************************************************************
    *  @brief: Value acquired by the Left oB Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double LoB;

    /***************************************************************************
    *  @brief: Value acquired by the Left B Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double LB;

    /***************************************************************************
    *  @brief: Value acquired by the Left iB Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double LiB;

    /***************************************************************************
    *  @brief: Value acquired by the Left ioB Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double LioB;

    /***************************************************************************
    *  @brief: Value acquired by the Left iMm Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double LiMm;

    /***************************************************************************
    *  @brief: Value acquired by the Left Mm Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double LMm;

    /***************************************************************************
    *  @brief: Value acquired by the Left oMl Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double LoMl;

    /***************************************************************************
    *  @brief: Value acquired by the Left S Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double LS;

    /***************************************************************************
    *  @brief: Value acquired by the Right oB Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double RoB_input;

    /***************************************************************************
    *  @brief: Value acquired by the Right B Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double RB_input;

    /***************************************************************************
    *  @brief: Value acquired by the Right iB Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double RiB_input;

    /***************************************************************************
    *  @brief: Value acquired by the Right ioB Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double RioB_input;

    /***************************************************************************
    *  @brief: Value acquired by the Right iMm Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double RiMm_input;

    /***************************************************************************
    *  @brief: Value acquired by the Right Mm Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double RMm_input;

    /***************************************************************************
    *  @brief: Value acquired by the Right oMl Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double RoMl_input;

    /***************************************************************************
    *  @brief: Value acquired by the Right S Direction Selective (DS) cell in 
    *          later PT before being passed through a Sigmoid function
    ***************************************************************************/
    double RS_input;

    /***************************************************************************
    *  @brief: Value acquired by the Right oB Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double RoB;

    /***************************************************************************
    *  @brief: Value acquired by the Right B Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double RB;

    /***************************************************************************
    *  @brief: Value acquired by the Right iB Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double RiB;

    /***************************************************************************
    *  @brief: Value acquired by the Right ioB Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double RioB;

    /***************************************************************************
    *  @brief: Value acquired by the Right iMm Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double RiMm;

    /***************************************************************************
    *  @brief: Value acquired by the Right Mm Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double RMm;

    /***************************************************************************
    *  @brief: Value acquired by the Right oMl Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double RoMl;

    /***************************************************************************
    *  @brief: Value acquired by the Right S Direction Selective (DS) cell in 
    *          later PT after being passed through a Sigmoid function
    ***************************************************************************/
    double RS_;

    /***************************************************************************
    *  @brief: Value acquired by the Left nMLF (LMLF) before being passed 
    *          through a Sigmoid function
    ***************************************************************************/
    double PTLMLF_Naumann;

    /***************************************************************************
    *  @brief: Value acquired by the Right nMLF (RMLF) before being passed 
    *          through a Sigmoid function
    ***************************************************************************/
    double PTRMLF_Naumann;

    /***************************************************************************
    *  @brief: Value acquired by the Left nMLF (LMLF) after being passed 
    *          through a Sigmoid function
    ***************************************************************************/
    double LMLF_Naumann;

    /***************************************************************************
    *  @brief: Value acquired by the Right nMLF (RMLF) after being passed 
    *          through a Sigmoid function
    ***************************************************************************/
    double RMLF_Naumann;
    
    /***************************************************************************
    *  @brief: Value acquired by the Left aHB (aHB) before being passed 
    *          through a Sigmoid function
    ***************************************************************************/
    double PTLaHB;

    /***************************************************************************
    *  @brief: Value acquired by the Right nMLF (RMLF) before being passed 
    *          through a Sigmoid function
    ***************************************************************************/
    double PTRaHB;

    /***************************************************************************
    *  @brief: Value acquired by the Left LHB (LLHB)
    ***************************************************************************/
    double LLHB;
    
    /***************************************************************************
    *  @brief: Value acquired by the Right LHB (RLHB)
    ***************************************************************************/
    double RLHB;


    public:

    /***************************************************************************
    *  @brief: Default constructor
    ***************************************************************************/
    LaterPT();

    /***************************************************************************
    *  @brief: Updates the values of Left nMLF (LMLF) and Right nMLF (RMLF)
    *
    *  @details: Updates the values of the nMLF (both Right nMLF and Left nMLF)
    *            starting from the output values of the four Direction Selective
    *            Ganglion Cells (smoothed and passed through the sigmoid function)
    *            that are passed to the function as parameters
    ***************************************************************************/
    void UpdateMLF(double LPT_R_0, double LPT_R_1, double LPT_R_2, double LPT_R_3, double RPT_L_0, double RPT_L_1, double RPT_L_2, double RPT_L_3);
    
    /***************************************************************************
    *  @brief: Updates the values of Left LHB (LLHB) and Right LHB (RLHB)
    *
    *  @details: Updates the values of the LHB (both Right LHB and Left LHB)
    *  starting from the output values of the Direction Selective (DS) left and
    *  right cells in later PT
    ***************************************************************************/
    void UpdateLHB();

    /***************************************************************************
    *  @brief: Returns the most recent value of nMLF
    *
    *  @details: Returns the most recent value of nMLF. To discriminate whether
    *            the Left nMLF value (LMLF) or the Right nMLF value (RMLF)
    *            should be returned, a direction flag is passed to the function
    ***************************************************************************/
    double GetMLF(int direction_flag);
    
    /***************************************************************************
    *  @brief: Returns the most recent value of LHB
    *
    *  @details: Returns the most recent value of LHB.To discriminate whether
    *            the Left LHB value (LLHB) or the Right LHB (RLHB) value should
    *            be returned, a direction flag is passed to the function
    ***************************************************************************/
    double GetLHB(int direction_flag);

    /***************************************************************************
    *  @brief: Returns the most recent value of LOB
    ***************************************************************************/ 
    double GetLOB();

    /***************************************************************************
    *  @brief: Returns the most recent value of LB
    ***************************************************************************/ 
    double GetLB();

    /***************************************************************************
    *  @brief: Returns the most recent value of LIB
    ***************************************************************************/ 
    double GetLIB();

    /***************************************************************************
    *  @brief: Returns the most recent value of LIOB
    ***************************************************************************/
    double GetLIOB();

    /***************************************************************************
    *  @brief: Returns the most recent value of LIMM
    ***************************************************************************/ 
    double GetLIMM();

    /***************************************************************************
    *  @brief: Returns the most recent value of LMM
    ***************************************************************************/ 
    double GetLMM();

    /***************************************************************************
    *  @brief: Returns the most recent value of LOML
    ***************************************************************************/ 
    double GetLOML();

    /***************************************************************************
    *  @brief: Returns the most recent value of LS
    ***************************************************************************/ 
    double GetLS();

    /***************************************************************************
    *  @brief: Returns the most recent value of ROB
    ***************************************************************************/ 
    double GetROB();

    /***************************************************************************
    *  @brief: Returns the most recent value of RB
    ***************************************************************************/ 
    double GetRB();

    /***************************************************************************
    *  @brief: Returns the most recent value of RIB
    ***************************************************************************/ 
    double GetRIB();

    /***************************************************************************
    *  @brief: Returns the most recent value of RIOB
    ***************************************************************************/ 
    double GetRIOB();

    /***************************************************************************
    *  @brief: Returns the most recent value of RIMM
    ***************************************************************************/ 
    double GetRIMM();

    /***************************************************************************
    *  @brief: Returns the most recent value of RMM
    ***************************************************************************/ 
    double GetRMM();

    /***************************************************************************
    *  @brief: Returns the most recent value of ROML
    ***************************************************************************/ 
    double GetROML();

    /***************************************************************************
    *  @brief: Returns the most recent value of RS
    ***************************************************************************/ 
    double GetRS();

};

#endif





