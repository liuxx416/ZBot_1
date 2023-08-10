#ifndef BEHAVIOURDETERMINATOR_H
#define BEHAVIOURDETERMINATOR_H

#include <iostream>
#include <math.h>
#include "AD.hpp"

// Probability offsets
#define left_prob_offset 0.066
#define right_prob_offset 0.066
#define forward_prob_offset 0.2

class BehaviourDeterminator {

    private:
    /***************************************************************************
    *  @brief: A log file to save the results of behaviour determinator.
    ***************************************************************************/
    FILE * log_file_BEHAVIOUR;

    /***************************************************************************
    *  @brief: Average between the values assumed by the Left nMLF (LMLF) and
    *          by the Right nMLF (RMLF).
    ***************************************************************************/
    double LRMLF;

    /***************************************************************************
    *  @brief: Random number that will decide the robot's behaviour.
    ***************************************************************************/
    double random_command;

    /***************************************************************************
    *  @brief: Sum of the probability offsets.
    ***************************************************************************/
    double sum_prob_offset;

    /***************************************************************************
    *  @brief: Command that indicates the behaviour chosen by the robot.
    *          The possible behaviours are a left turn (command = -1), a right
    *          turn (command = 1) or a forward bout (command = 0).
    ***************************************************************************/
    int command;

    /***************************************************************************
    *  @brief: Probability of a left turn.
    ***************************************************************************/
    double probability_LT;

    /***************************************************************************
    *  @brief: Probability of a right turn.
    ***************************************************************************/
    double probability_RT;

    /***************************************************************************
    *  @brief: Probability of a forward bout.
    ***************************************************************************/
    double probability_FB;

    /***************************************************************************
    *  @brief: Tail phase.
    ***************************************************************************/
    double phase;

    /***************************************************************************
    *  @brief: Flag that indicates whether the first tail segment is bending
    *          towards the left (first_bend_flag = 0) or towards the right
    *          (first_bend_flag = 1)
    ***************************************************************************/
    bool first_bend_flag = 0;

    /***************************************************************************
    *  @brief: An AD object
    ***************************************************************************/   
    //AD ADinputs;

    public:

    /***************************************************************************
    *  @brief: Default constructor
    ***************************************************************************/
    BehaviourDeterminator();

    /***************************************************************************
    *  @brief: Default deconstructor
    ***************************************************************************/
    ~BehaviourDeterminator();

    /***************************************************************************
    *  @brief: Determines the behaviour of the robot.
    *
    *  @details: The function determines the behaviour of the robot starting
    *            from the values assumed by the Left nMLF (LMLF), by the Right
    *            nMLF (RMLF), by the Left LHB (LLHB) and by the Right LHB (RLHB).
    *            In particular, the possible behaviours are: execution of a left
    *            turn, execution of a forward bout and execution of a right turn.
    *            The probabilities associated with the three behaviours are also
    *            computed and displayed.
    ***************************************************************************/
    void DetermineBehaviour(double LMLF, double RMLF, double LLHB, double RLHB);

    /***************************************************************************
    *  @brief: Gets the behaviour selected by the robot.
    *
    *  @details: This function returns the behaviour selected by the robot.
    *            In particular, it returns value -1 if the robot is turning left;
    *            it returns value 0 if the robot is swimming forward; it returns
    *            value 1 if the robot is turning right.
    ***************************************************************************/
    int GetCommand();

    /***************************************************************************
    *  @brief: Gets the tail phase.
    *
    *  @details: This function returns the phase of the robot's tail.
    ***************************************************************************/
    double GetPhase();

    /***************************************************************************
    *  @brief: Gets a flag that indicates how the robot's tail is bending.
    *
    *  @details: This function returns a flag that indicates whether the first
    *            tail segment is bending to the left or to the right: it returns
    *            value 0 if the first tail segment is bending to the left; it
    *            returns value 1 if the first tail segment is bending to the
    *            right.
    ***************************************************************************/
    bool GetBendFlag();
   
    void CloseLogFile() {fclose(log_file_BEHAVIOUR);}
};

#endif
