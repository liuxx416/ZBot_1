#include "behaviourDeterminator.hpp"

// Default constructor
BehaviourDeterminator::BehaviourDeterminator()
{
  char log_file_BEHAVIOUR_name[255];
  struct tm* tm;
  time_t now;
  now = time(0); // get current time
  tm = localtime(&now); // get structure
  sprintf(log_file_BEHAVIOUR_name, "logFiles/log_file_BEHAVIOUR%04d%02d%02d%02d%02d%02d", tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
  log_file_BEHAVIOUR = fopen(log_file_BEHAVIOUR_name, "w");
}

BehaviourDeterminator::~BehaviourDeterminator()
{
  fclose(log_file_BEHAVIOUR);
}

// Helper function that computes a random number starting from a gaussian distribution
double rand_gaussrand();

// This function, starting from the values assumed by the Left nMLF (LMLF), by the Right nMLF (RMLF), by the Left LHB (LLHB) and
// by the Right LHB (RLHB) determines the behaviour of the robot. In particular, the possible behaviours are: execution of a left
// turn, execution of a forward bout and execution of a right turn. The probabilities associated with the three behaviours are also
// computed and displayed.
void BehaviourDeterminator::DetermineBehaviour(double LMLF, double RMLF, double LLHB, double RLHB)
{
    // The average between the value assumed by the Left nMLF and the value assumed by the Right nMLF is computed
    LRMLF = (LMLF + RMLF) / 2;
    // A random number is drawn. This random number will decide the robot's behaviour, taking into account also the probabilities
    // associated with each type of behaviour
    random_command = rand() / (RAND_MAX + 1.0);
    sum_prob_offset = left_prob_offset + right_prob_offset + forward_prob_offset;

    if (random_command < ((LLHB + left_prob_offset) / (LRMLF  * 2.0 + LLHB + RLHB + sum_prob_offset)))
    {
      command = -1; // Turn left
    }
    else if (random_command < ((LRMLF * 2.0 + LLHB + forward_prob_offset + left_prob_offset) / (LRMLF  * 2.0 + LLHB + RLHB + sum_prob_offset)))
    {
      command = 0; // Swim forward
    }
    else if (random_command < ((LRMLF * 2.0 + LLHB + RLHB + sum_prob_offset) / (LRMLF  * 2.0 + LLHB + RLHB + sum_prob_offset)))
    {
      command = 1; // Turn right
    }
    // command = 0;// For bout test, will be deleted later
    // The probabilities associated with each type of behaviour are computed and displayed
    probability_LT = (LLHB + left_prob_offset) / (LRMLF  * 2.0 + LLHB + RLHB + sum_prob_offset);
    probability_FB = (LRMLF * 2.0 + forward_prob_offset) / (LRMLF  * 2.0 + LLHB + RLHB + sum_prob_offset);
    probability_RT = (RLHB + right_prob_offset) / (LRMLF  * 2.0 + LLHB + RLHB + sum_prob_offset);
    std::cout << " Probability of a left turn: " << probability_LT << std::endl;
    std::cout << " Probability of a forward bout: " << probability_FB << std::endl;
    std::cout << " Probability of a right turn: " << probability_RT << std::endl;
    // The command chosen is displayed
    std::cout << " Command chosen: " << command << " (-1 -> turn left | 0 -> swim forward | 1 -> turn right)" << std::endl;

    //A log file to record the possibility of behaivour and command
    fprintf(log_file_BEHAVIOUR, "%0.2f \t %0.2f \t %0.2f \t %d \n", probability_LT, probability_FB, probability_RT, command);
    //ADinputs.refreshADInputs();
    //float ADValues[4] = {0, 0, 0, 0};
    //ADValues[0] = ADinputs.getADInputs(0);
    //ADValues[1] = ADinputs.getADInputs(1); 
    //ADValues[2] = ADinputs.getADInputs(2); 
    //ADValues[3] = ADinputs.getADInputs(3);

    //if (ADValues[1] < 30) command = 0;		//command == 0: forward bout
    //else if (ADValues[0] < 30) command = -1;	//command == -1: leftward bout
    //else if (ADValues[0] > 70) command = 1;	//command == 1: rightward bout
    //else if (ADValues[2] > 70) command = 2;	//command == 2: stop
    //else command = command;
    //std::cout << "AD0 AD1 AD2 AD3 Command **********************************************************************************: " << ADValues[0] << " " << ADValues[1] << " " << ADValues[2] << " " << ADValues[3] << " " << command << std::endl;

    // The following part is based on [Huang et al. 2013] and his PhD thesis.
    // If the command is to turn left, the first tail segment bend to the left
    // If the command is to turn right, the first tail segment bend to the right
    // If the command is to swim forward, the first tail segment bend to the left or to the right (according to the value obtained using rand_gaussrand())
    if (command == -1) // The robot is turning left
    {
      phase = M_PI;
      // Setting the 'first_bend_flag' flag to 0 indicates that the first tail segment is bending to the left
      first_bend_flag = 0;
    }
    else if (command == 1) // The robot is turning right
    {
      phase = 0;
      // Setting the 'first_bend_flag' flag to 1 indicates that the first tail segment is bending to the right
      first_bend_flag = 1;
    }
    else if (command == 0 && rand_gaussrand() < 0) // The robot is swimming forward
    {
      phase = M_PI;
      first_bend_flag = 0;
    }                           
    else // The robot is swimming forward
    {
      phase = 0;
      first_bend_flag = 1;
    }
}

// This function returns the behaviour selected by the robot:
// the variable that is returned assumes value -1 if the robot is turning left
// the variable that is returned assumes value  0 if the robot is swimming forward
// the variable that is returned assumes value  1 if the robot is turning right
int BehaviourDeterminator::GetCommand()
{
    return command;
}

// This function returns the tail phase computed
double BehaviourDeterminator::GetPhase()
{
    return phase;
}

// This function returns a flag that indicates whether the first tail segment is bending to the left or to the right:
// the variable that is returned assumes value 0 if the first tail segment is bending to the left
// the variable that is returned assumes value 1 if the first tail segment is bending to the right
bool BehaviourDeterminator::GetBendFlag()
{
    return first_bend_flag;
}

// Helper function that computes a random number starting from a gaussian distribution
double rand_gaussrand()
{
	double V1 = 0, V2 = 0, S = 0, X1 = 0;
	int phase3 = 0;

	if (phase3 == 0)
  {
		do
    {
			float U1 = (float) rand() / RAND_MAX;
			float U2 = (float) rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		}
    while (S >= 1 || S == 0);

		X1 = V1 * sqrt(-2 * log(S) / S);
	}
  else
  {
    X1 = V2 * sqrt(-2 * log(S) / S);
  }

	phase3 = 1 - phase3;

	return X1;
}




    
