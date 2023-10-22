
#include "envirobot.hpp"
#include <iomanip>
#include <bcm2835.h>

// Mutex instructions to avoid crashes
#include <mutex>
std::mutex mutex_img_access;
std::mutex mutex_LMLF;
std::mutex mutex_RMLF;
std::mutex mutex_LLHB;
std::mutex mutex_RLHB;

// Mutex parameters of assistance thread
std::mutex mutex_ADInputValues;
//std::mutex ADInputValue0;
//std::mutex ADInputValue1;
//std::mutex ADInputValue2;
//std::mutex ADInputValue3;
std::mutex mutex_INA219;
int stop_from_GUI = 0;

// Lines setting up the server
// The output image will be the the middle half of the rows of the two images placed side by size, but scaled down by 2.
cs::CvSource cvsource{"cvsource", cs::VideoMode::kMJPEG, ROI_HEIGHT*2, ROI_WIDTH*2*2, 30};
cs::MjpegServer cvMjpegServer{"cvhttpserver", 8080};


// Helper function that returns a if a is between b and c. If a is smaller than b, the
// function returns b, if a is greater than c, the function returns c
float clamp(float a, float b, float c);

// Helper function to determine the maximum between four float values and to normalize
// the four values. After the normalization, the largest variable assumes the value 100.0
void normal_max(double *fl_1, double *fl_2, double *fl_3, double *fl_4);

// Helper function that computes the result of the sigmoid function given omega, the bias and the input value.
// The functions apply the sigmoid function (defined by the parameters omega and bias) on the input value 'in'
double Sigmoid(double omega, double bias, double in);

// Helper function that computes a random number starting from a Gaussian distribution
double rand_gaussianrand();

// Helper function that converts the motor position to a value suitable for the motor currently in use
std::vector<int> ConvertMotorPosition(std::vector<double> INPosition);

// Helper function that computes and returns the time elapsed (in milliseconds) from the time_point previousEventTime
double TimeElapsed_(std::chrono::high_resolution_clock::time_point previousEventTime)
{
	std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
	float t = std::chrono::duration_cast<std::chrono::microseconds> (currentTime - previousEventTime).count();
	return t/1000;
}

// Helper function used to execute commands from command line and obtain the result in form of a string
// Source: https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-the-output-of-the-command-within-c-using-po
std::string execute(const char* cmd);

// Starting point of the clock
std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();


/******************************************************************************/

// Default constructor
Envirobot::Envirobot():
                      d_eyes(1),
                      d_mot(MOTOR_NUM)

{
}

/******************************************************************************/

// Custom constructor
Envirobot::Envirobot(std::vector<std::pair<Orientation, std::string>> cameraInfo):
                      d_eyes(cameraInfo.size()),
                      d_mot(MOTOR_NUM)
{

  // Assign a serial number and orientation to each of the eyes
  for(unsigned int i = 0; i < cameraInfo.size(); i++)
  {
    // Extract the camera orientation and serial number from the pair object
    Orientation cameraOrientation = cameraInfo[i].first;
    std::string cameraSerialNumber = cameraInfo[i].second;

    // Assign the properties to each eye
    d_eyes[i].SetSerialNumber(cameraSerialNumber);
    d_eyes[i].SetOrientation(cameraOrientation);
  }
}

/******************************************************************************/

// Destructor
Envirobot::~Envirobot()
{
}

/******************************************************************************/

// This function initializes all the eyes of the robot and starts the image
// acquisition for each of the eyes
void Envirobot::OpenEyes(int GUI_FLAG)
{
  for(auto iter_eye = d_eyes.begin(); iter_eye != d_eyes.end(); iter_eye++)
  {
    iter_eye->OpenEye(GUI_FLAG);
  }
}

/******************************************************************************/

// This function initializes all motors of the robot.
// If the motors connection has been successful, the function returns the value 1,
// otherwise it returns the value 0.
int Envirobot::OpenMotors()
{
  int motor_id = 0;
  int result = 0;
  int success = 1;
  for(auto iter_mot = d_mot.begin(); iter_mot != d_mot.end(); iter_mot++)
  {
    result = iter_mot->InitializeMotor(motor_id);
    motor_id = motor_id + 1;
    if (result == 0)
    {
      success = 0;
    }
  }

  return success;
}

/******************************************************************************/

// This function stops the image acquisition for each of the eyes and closes all
// eyes of the envirobot
void Envirobot::CloseEyes()
{
  for(auto iter_eye = d_eyes.begin(); iter_eye != d_eyes.end(); iter_eye++)
  {
    iter_eye->CloseEye();
  }
}

/******************************************************************************/

// This function terminates all motors of the robot
void Envirobot::CloseMotors()
{
  int motor_id = 0;
  int last = 0;
  
  for(auto iter_mot = d_mot.begin(); iter_mot != d_mot.end(); iter_mot++)
  {
    if (motor_id == (MOTOR_NUM - 1))
    {
      last = 1;
    }
    iter_mot->TerminateMotor(motor_id, last);
    motor_id = motor_id + 1;
  }
}

/******************************************************************************/

// This function integrates the assistance functions such as remote control and other sensory inputs from HAT sensor module
void Envirobot::AssistanceThread(int startup, int ex_time_s, int GUI_FLAG, std::string IPv4_to_contact)
{
  long int counter = 0;
  std::chrono::high_resolution_clock::time_point globalStartTime = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point localStartTime = std::chrono::high_resolution_clock::now();
  float avg_time = 0;

  int PIN31_counter = 0;
  int PIN33_counter = 0;
  int PIN35_counter = 0;
  int PIN37_counter = 0;
  int overAll_counter =0;

  #define PIN31 RPI_BPLUS_GPIO_J8_31
  #define PIN33 RPI_BPLUS_GPIO_J8_33
  #define PIN35 RPI_BPLUS_GPIO_J8_35
  #define PIN37 RPI_BPLUS_GPIO_J8_37

//   #define PIN31 RPI_V2_GPIO_P1_31
//   #define PIN33 RPI_V2_GPIO_P1_33
//   #define PIN35 RPI_V2_GPIO_P1_35
//   #define PIN37 RPI_V2_GPIO_P1_37

    if (!bcm2835_init())
	return;

//    bcm2835_gpio_fsel(PIN31, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(PIN33, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(PIN35, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(PIN37, BCM2835_GPIO_FSEL_INPT);
 //   bcm2835_gpio_set_pud(PIN31, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(PIN33, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(PIN35, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(PIN37, BCM2835_GPIO_PUD_UP);
 //   bcm2835_gpio_hen(PIN31);
 //   bcm2835_gpio_hen(PIN33);
 //   bcm2835_gpio_hen(PIN35);
 //   bcm2835_gpio_hen(PIN37);
  do
  {
    float time_elapsed = TimeElapsed_(startTime);
    std::this_thread::sleep_for(std::chrono::milliseconds(200 - int(time_elapsed)));
    startTime = std::chrono::high_resolution_clock::now();

    avg_time = ((counter+1)*avg_time + time_elapsed)/(counter+2);
    PIN31_counter = 0;
    PIN33_counter = 0;
    PIN35_counter = 0;
    PIN37_counter = 0;
    overAll_counter = 0;
    localStartTime = std::chrono::high_resolution_clock::now();
        do{
        // Count the HIGH inputs
        PIN31_counter+=bcm2835_gpio_lev(PIN31);
        PIN33_counter+=bcm2835_gpio_lev(PIN33);
        PIN35_counter+=bcm2835_gpio_lev(PIN35);
        PIN37_counter+=bcm2835_gpio_lev(PIN37);
        overAll_counter++;
        bcm2835_delayMicroseconds(200);//delay 200 us	
        }while (TimeElapsed_(localStartTime) < 190);

	//invert the channels, because PIN31 responses too slow to detect the RC receiver
	//Actually, we are noting using the PIN31,
    float CH4 = (PIN31_counter * 10.0 / overAll_counter - 0.7) * 160;
    float CH3 = (PIN33_counter * 10.0 / overAll_counter - 0.7) * 160;
    float CH2 = (PIN35_counter * 10.0 / overAll_counter - 0.7) * 160;
    float CH1 = (PIN37_counter * 10.0 / overAll_counter - 0.7) * 160;
    
    mutex_ADInputValues.lock();
    ADInputValues[0] = CH1;
    ADInputValues[1] = CH2; 
    ADInputValues[2] = CH3; 
    ADInputValues[3] = CH4;
    mutex_ADInputValues.unlock();
    counter = counter + 1;   
//    std::cout <<" Time in simulation ms: " << TimeElapsed_(globalStartTime) << " overAll_counter: " << overAll_counter << " CH1: " << CH1 << " CH2: " << CH2 << " CH3: " << CH3 << " CH4:" << CH4 << std::endl;
  }
  while ((!startup) && (TimeElapsed_(globalStartTime) < ex_time_s*1000) && (stop_from_GUI == 0));
  std::cout << " AssistantThread finish " << std::endl;
}

void Envirobot::PowerMeterThread(int startup, int ex_time_s, int GUI_FLAG, std::string IPv4_to_contact)
{
  long int counter = 0;
  std::chrono::high_resolution_clock::time_point globalStartTime = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
  float avg_time = 0;
  INA219 PowerMeter_temp(0.01, 8);
  PowerMeter_temp.configure(RANGE_16V, GAIN_8_320MV, ADC_4SAMP, ADC_4SAMP);
 // PowerMeter.configure(RANGE_16V, GAIN_8_320MV, ADC_10BIT, ADC_10BIT);

  do
  {
    // Uncomment the following line to print debug and log information
    // std::cout << "UpdateVisualInformation (eye " << eye->EyeOrientation() << ") with ID " << counter;
    float time_elapsed = TimeElapsed_(startTime);
    std::this_thread::sleep_for(std::chrono::milliseconds(20 - int(time_elapsed)));    
    startTime = std::chrono::high_resolution_clock::now();
    avg_time = ((counter+1)*avg_time + time_elapsed)/(counter+2);

    //Refresh the AD inputs
   mutex_INA219.lock();
    PowerMeterOutput[0] = PowerMeter_temp.power();
    PowerMeterOutput[1] = PowerMeter_temp.voltage();
    PowerMeterOutput[2] = PowerMeter_temp.current();    
    mutex_INA219.unlock();
    fprintf(log_file_PowerMeter, "%0.6f \t %0.6f \t %0.3f \t %0.6f \n", TimeElapsed_(globalStartTime)/1000, PowerMeterOutput[0]/1000, PowerMeterOutput[1], PowerMeterOutput[2]/1000);
//     fprintf(log_file_PowerMeter, "%0.3f \t %0.3f \t %0.3f \t %0.3f \n", TimeElapsed_(globalStartTime), PowerMeterOutput[0], PowerMeterOutput[1], PowerMeterOutput[2]);    
    // Uncomment the following line to print debug and log information
//    std::cout << " Time in simulation ms: " << TimeElapsed_(globalStartTime) << "Average loop time: " << avg_time << ")" << std::endl;
    //std::cout << " Power meter " << PowerMeterOutput[0] << std::endl;
    //startTime = std::chrono::high_resolution_clock::now();
        
    counter = counter + 1;   
  }
  while ((!startup) && (TimeElapsed_(globalStartTime) < ex_time_s*1000) && (stop_from_GUI == 0));
  std::cout << " PowerMeterThread finish " << std::endl;
}

/******************************************************************************/

// This function updates the visual information for the eye passed as a parameter
void Envirobot::UpdateVisualInformation(Eye* eye, int startup, int ex_time_s, int GUI_FLAG, std::string IPv4_to_contact)
{
  long int counter = 0;
  std::chrono::high_resolution_clock::time_point globalStartTime = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
  float avg_time = 0;

  do
  {
    // Uncomment the following line to print debug and log information
    // std::cout << "UpdateVisualInformation (eye " << eye->EyeOrientation() << ") with ID " << counter;
    float time_elapsed = TimeElapsed_(startTime);
    startTime = std::chrono::high_resolution_clock::now();
    avg_time = ((counter+1)*avg_time + time_elapsed)/(counter+2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100 - int(time_elapsed)));

    // Uncomment the following line to print debug and log information
    //std::cout << " (Elapsed time from last in ms: " << time_elapsed << "- Average elapsed time from last in ms: " << avg_time << ")" << std::endl;
    //startTime = std::chrono::high_resolution_clock::now();
    
    counter = counter + 1;
    
    // Get new image/images using the eye that was passed as a parameter
    eye->GetNewImages();
    
  }
  while ((!startup) && (TimeElapsed_(globalStartTime) < ex_time_s*1000) && (stop_from_GUI == 0));

}

/******************************************************************************/

// This function computes the target position of the motors and moves all the motors of the robot
void Envirobot::MoveMotors(int startup, int ex_time_s)
{
  long int counter = 0;
  int index_motor = 0;
  std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point globalStartTime = std::chrono::high_resolution_clock::now();
  float avg_time = 0;
  double last_swim_amplitude;
  double last_left_swim_amplitude;
  double last_right_swim_amplitude;
  double current_swim = 0;
  double left_descending_amplitude = 0;
  double right_descending_amplitude = 0;
  double last_LVSPNs;
  double last_RVSPNs;
  double current_LVSPNs = 0;
  double current_RVSPNs = 0;
  double left_amplitude = 0;
  double right_amplitude = 0;
 // bool bout_flag = 0; // 0 = no bout | 1 = bout
  float control_step_fixed = 20;
  


  do
  {
    // Uncomment the following line to print debug and log information
    // std::cout << "MoveMotors with ID " << counter;
    float time_elapsed = TimeElapsed_(startTime);
    avg_time = ((counter+1)*avg_time + time_elapsed)/(counter+2);
    // Uncomment the following line to print debug and log information
    // std::cout << " (Elapsed time from last in ms: " << time_elapsed << "- Average elapsed time from last in ms: " << avg_time << ")" << std::endl;    
    std::this_thread::sleep_for(std::chrono::milliseconds(20 - int(time_elapsed)));
    startTime = std::chrono::high_resolution_clock::now();
    
    counter = counter + 1;
    
    current_swim = 0;
    left_descending_amplitude = 0;
    right_descending_amplitude = 0;
    current_LVSPNs = 0;
    current_RVSPNs = 0;
//    left_amplitude = 0;
//    right_amplitude = 0;
    bout_flag = 0;
  
    last_swim_amplitude = swim_amplitude;
    last_left_swim_amplitude =  left_swim_amplitude;
    last_right_swim_amplitude =  right_swim_amplitude;
    last_LVSPNs = LVSPNs_output;
    last_RVSPNs = RVSPNs_output;
    
      //Since there are multiple operations on the ADinputvalues in the following
  //Set a local parameter ADInputValues_Motor
    int ADInputValues_Motor[4];
    mutex_ADInputValues.lock();
    ADInputValues_Motor[0] = ADInputValues[0]; //Left and right turning
	ADInputValues_Motor[1] = ADInputValues[1]; //Bout frequency
	ADInputValues_Motor[2] = ADInputValues[2]; //Move or stop 
	ADInputValues_Motor[3] = ADInputValues[3]; 
    mutex_ADInputValues.unlock();
    //std::cout << "AD0 AD1 AD2 AD3 Command **********************************************************************************: " << ADInputValues_Motor[0] << " " << ADInputValues_Motor[1] << " " << ADInputValues_Motor[2] << " " << ADInputValues_Motor[3] << " " << command << std::endl;

    int SwimmingMode = 0;
    if (ADInputValues_Motor[2] > 125 || ADInputValues_Motor[2] < -25) SwimmingMode = 0; 			//Strange inputs, stop the robot
    else if (ADInputValues_Motor[2] > 70) SwimmingMode = 0; 										//Stop
    else if (ADInputValues_Motor[2] > 30) SwimmingMode = 1; 										//Bout and gliding swimming  
    else if (ADInputValues_Motor[2] > -25)  SwimmingMode = 2;											//Continues swimming
    else SwimmingMode = 0;
    
  if (SwimmingMode == 1)
  phase = phase - 1.00  * (double)control_step_fixed / 1000.0 * 2.0 * M_PI;
  else if (SwimmingMode == 2)
  {  
  float AmpPhase = 0.3 * (50 - ADInputValues_Motor[1])/50 + 0.7;
  AmpPhase = clamp (AmpPhase, 0, 1);
  phase = phase - 1 * (double)control_step_fixed / 1000.0 * AmpPhase * 2.0 * M_PI;
  }
  else phase = phase;
  
  
  if (phase < -2000000)
  {
  phase = 0;
  }

if (SwimmingMode == 1)
{
    // Shishi-odoshi modules added for burst-gliding swimming.
    // The following line has been used while performing the experiments (to make the bouts more frequent)
    if (SS_MLF > 160) //20 ->  1s
    //if (SS_MLF > 255)
    {
      // The behaviour of the robot is determined taking into account the values assumed by LMLF, RMLF, LLHB, RLHB
      // Mutex instructions to avoid crashes
      mutex_LMLF.lock();
      mutex_RMLF.lock();
      mutex_LLHB.lock();
      mutex_RLHB.lock();
      behaviour_det.DetermineBehaviour(LMLF, RMLF, LLHB, RLHB);
      mutex_LMLF.unlock();
      mutex_RMLF.unlock();
      mutex_LLHB.unlock();
      mutex_RLHB.unlock();

      // The chosen behaviour is obtained
      command = behaviour_det.GetCommand();

      // The tail phase is obtained
      phase = behaviour_det.GetPhase();
      // A flag that indicates whether the first tail segment is bending to the left or to the right is obtained
      first_bend_flag = behaviour_det.GetBendFlag();

      //Add manual inputs for the direction of swimming
      if (ADInputValues_Motor[0] < 30) {command = -1;phase = M_PI;first_bend_flag = 0;} //Left-ward turning bout
      else if (ADInputValues_Motor[0] > 70) {command = 1;phase = 0;first_bend_flag = 1;} //Right-ward turning bout
      else if (ADInputValues_Motor[1] < 30) command = 0; //Forward bout, let the system randomly choose the phase and first_bend_flag
      else command = command; //Keep the command


      // Reset the Shishi-odoshi and count down
      noise_SS_MLF_deduction_rate =  0;//clamp(rand_gaussianrand() * 0.0005, -0.001 , 0.001);
      SS_MLF = 0;
      SS_MLF_countdown = 500; 
      SS_LVSPNs_countdown = 500;
      SS_RVSPNs_countdown = 500;
	//to get standard bouting behaviour, we apply no noise tothe neurons
      noise_turning_LSPN = 0 * rand_gaussianrand() * noise_to_SPNs; 
      noise_turning_RSPN = 0 * rand_gaussianrand() * noise_to_SPNs;
      noise_forward_LSPN = 0 * rand_gaussianrand() * noise_to_SPNs;
      noise_forward_RSPN = 0 * rand_gaussianrand() * noise_to_SPNs; 
    }
    else if (SS_MLF_countdown <= 5) // The robot is stopping/gliding, accumulate the Shishi-odoshis
    {
      double random_n = abs(rand_gaussianrand()) * 0.18;

      // Mutex instructions to avoid crashes
      mutex_LMLF.lock();
      mutex_RMLF.lock();
      SS_MLF = SS_MLF + ((LMLF * 0.5 + RMLF * 0.5) * 1.2)  + 0.2;//random_n;
      mutex_LMLF.unlock();
      mutex_RMLF.unlock();

      SS_MLF = SS_MLF * 1; // Change the value currently set to 1 to add leaky effects
    }

    if (SS_MLF_countdown > 46) // Bout action in progress
    {
      bout_flag = 1;
      SS_MLF_countdown = SS_MLF_countdown * (0.9772 + noise_SS_MLF_deduction_rate);
      current_swim = SS_MLF_countdown / 400;
      left_descending_amplitude = 1.0 * SS_MLF_countdown / 400; 
      right_descending_amplitude = 1.0 * SS_MLF_countdown / 400;

      if (command == -1) // The robot is turning left
      {
        // The values of the ventromedial Spinal Projection Neurons (vSPNs) are computed
        // taking into account that the robot is turning left
        current_LVSPNs = (1.0 + noise_turning_LSPN) * SS_LVSPNs_countdown / 400;
        current_RVSPNs = (0.0 + noise_turning_RSPN) * SS_RVSPNs_countdown / 400;
      }
      else if (command == 0) // The robot is swimming forward
      {
        // The values of the ventromedial Spinal Projection Neurons (vSPNs) are computed
        // taking into account that the robot is swimming forward
        current_LVSPNs = (0.0 + noise_forward_LSPN) * SS_LVSPNs_countdown / 400;
        current_RVSPNs = (0.0 + noise_forward_RSPN) * SS_RVSPNs_countdown / 400;
      }
      else if (command == 1) // The robot is turning right
      {
        // The values of the ventromedial Spinal Projection Neurons (vSPNs) are computed
        // taking into account that the robot is turning right
        current_LVSPNs = (0.0 + noise_turning_LSPN) * SS_LVSPNs_countdown / 400;
        current_RVSPNs = (1.0 + noise_turning_RSPN) * SS_RVSPNs_countdown / 400;
      }

      SS_LVSPNs_countdown = SS_LVSPNs_countdown * 0.935;
      SS_RVSPNs_countdown = SS_RVSPNs_countdown * 0.935;
    }
   else if (SS_MLF_countdown > 40) //if the amplitude is small enough, reduce all amplitude to 0 (zero)
   {
      SS_MLF_countdown = SS_MLF_countdown * (0.9772 + noise_SS_MLF_deduction_rate);
      bout_flag = 1;
      current_swim = current_swim * 0.8;
      current_LVSPNs = current_LVSPNs * 0.8;
      current_RVSPNs = current_RVSPNs * 0.8;
      left_descending_amplitude = left_descending_amplitude * 0.8;
      right_descending_amplitude = right_descending_amplitude * 0.8;
   }
    else // Bout action not in progress
    {
      bout_flag = 0;
      SS_MLF_countdown = 0;
      current_swim = 0;
      current_LVSPNs = 0;
      current_RVSPNs = 0;
      left_descending_amplitude = 0;
      right_descending_amplitude = 0;
    }

    //Low Pass Filters to smooth the outputs
    swim_amplitude = last_swim_amplitude * 0.965 + current_swim * 0.035;
    left_swim_amplitude = last_left_swim_amplitude * 0.975 + left_descending_amplitude * 0.025;
    right_swim_amplitude = last_right_swim_amplitude * 0.975 + right_descending_amplitude * 0.025;   
    LVSPNs_output = last_LVSPNs * 0.95 + current_LVSPNs * 0.05;
    RVSPNs_output = last_RVSPNs * 0.95 + current_RVSPNs * 0.05;   
    left_amplitude = left_swim_amplitude;
    right_amplitude = right_swim_amplitude;
}

else if (SwimmingMode == 2) //Continues swimming
{
    float  ReluADInputValues_Motor1 = 0;
    if (ADInputValues_Motor[1] > 50)ReluADInputValues_Motor1 = 50;
    else ReluADInputValues_Motor1 = ADInputValues_Motor[1];

    left_amplitude = left_amplitude * 0.965 + (50 - ReluADInputValues_Motor1) * 0.012 * 0.035;
    right_amplitude = right_amplitude * 0.965 + (50 - ReluADInputValues_Motor1) * 0.012 * 0.035;
    //LMLF = 0; 
    //RMLF = 0;
    if (ADInputValues_Motor[0] - 50 >= 0)
    {
    	LVSPNs_output = LVSPNs_output * 0.965 - (ADInputValues_Motor[0] - 50) * 0.012 * 0.035;
    	RVSPNs_output = RVSPNs_output * 0.965 + (ADInputValues_Motor[0] - 50) * 0.012 * 0.035;
    }
    else if (ADInputValues_Motor[0] - 50 < 0)
    {
    	LVSPNs_output = LVSPNs_output * 0.965 - (ADInputValues_Motor[0] - 50) * 0.012 * 0.035;
    	RVSPNs_output = RVSPNs_output * 0.965 + (ADInputValues_Motor[0] - 50) * 0.012 * 0.035;
    }
    
    if (LVSPNs_output < 0) LVSPNs_output = 0;
    else if (LVSPNs_output > 0.5) LVSPNs_output = 0.5;
    if (RVSPNs_output < 0) RVSPNs_output = 0;
    else if (RVSPNs_output > 0.5) RVSPNs_output = 0.5; 
    
    bout_flag = 1;
    first_bend_flag = first_bend_flag;
    phase = phase;    
}

else //Other mode (e.g. mode 0, stop)
{
left_amplitude = 0;
right_amplitude = 0;
LVSPNs_output = 0;
RVSPNs_output = 0;
continue;
}

  fprintf(log_file_TailBeatingAmplitude, "%0.3f \t %0.3f \t %0.3f \t %0.3f \t %0.3f \n", TimeElapsed_(globalStartTime)/1000, left_amplitude, right_amplitude, LVSPNs_output, RVSPNs_output);


    if (counter % 10 == 0 && SwimmingMode == 1)
    std::cout << "AD0 AD1 AD2 AD3 Command bout_flag SwimmingMode 1  ***************************: " << ADInputValues_Motor[0] << " " << ADInputValues_Motor[1] << " " << ADInputValues_Motor[2] << " " << ADInputValues_Motor[3] << " " << command << std::endl;
    else if (counter % 10 == 0 && SwimmingMode == 2)
    std::cout << "AD0 AD1 AD2 AD3 Amplitude%0.2f LVSPNs RVSPNs SwimmingMode 2  ***************************: " << ADInputValues_Motor[0] << " " << ADInputValues_Motor[1] << " " << ADInputValues_Motor[2] << " " << ADInputValues_Motor[3] << " " << left_amplitude << " " << LVSPNs_output << " " << RVSPNs_output << " " << SwimmingMode << std::endl;


    // The target motor positions (in radians) for all the motors of the robot are computed
    // Mutex instructions to avoid crashes
    mutex_LMLF.lock();
    mutex_RMLF.lock();
    motor_CPG.UpdateCPG(left_amplitude, right_amplitude, LMLF, RMLF, LVSPNs_output, RVSPNs_output, bout_flag, first_bend_flag, phase);
    mutex_LMLF.unlock();
    mutex_RMLF.unlock();
    // The target motor positions (in radians) for all the motors of the robot are obtained
    target_motor_position = motor_CPG.GetMotorPosition();

    fprintf(log_file_MOTORS,  "%0.3f \t %0.5f \t %0.5f \t %0.5f \t %0.5f \t %0.5f \t %0.5f \n",TimeElapsed_(globalStartTime)/1000, target_motor_position[0], target_motor_position[1], target_motor_position[2], target_motor_position[3], target_motor_position[4], target_motor_position[5]);

    // This part of the code is used to obtain values that are then displayed
    // The tail angle is calculated by forward kinematics
    // UpdateCoordinates();
    // To display the tail position and the tail angle, uncomment the following lined
    //std::cout << " The tail position is : (" << tail_position_x << ", " << tail_position_y << ") | format: (x, y)" << std::endl;
    //std::cout << " The tail angle is: " << tail_angle << std::endl << std::endl;
    
    /// Convert the motor positions (in radians) to values readable by the motors
    goal_motor_position = ConvertMotorPosition(target_motor_position);
    
    // Move all the motors (executed only if all the motors of the robots have been successfully connected)
    if (motors_connected)
    {
      Motor::MoveAllMotors(goal_motor_position);
    }
  }
  while ((!startup) && (TimeElapsed_(globalStartTime) < ex_time_s*1000) && (stop_from_GUI == 0));
std::cout << " MoveMotorThread finish " << std::endl;
}

/******************************************************************************/

// Function that handles the communication with the Graphic User Interface (GUI)
// Note that this function is executed only if the code is run in GUI mode
void Envirobot::GUICommunication(int ex_time_s, std::string IPv4_to_contact)
{
  double LOB;
  double LB;
  double LIB;
  double LIOB;
  double LIMM;
  double LMM;
  double LOML;
  double LS;
  double ROB;
  double RB;
  double RIB;
  double RIOB;
  double RIMM;
  double RMM;
  double ROML;
  double RS;
  
  long int counter = 0;
  std::chrono::high_resolution_clock::time_point globalStartTime = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
  float avg_time = 0;
  
  do
  {
    // Uncomment the following line to print debug and log information
    // std::cout << "GUICommunication with ID " << counter;
    float time_elapsed = TimeElapsed_(startTime);
    avg_time = ((counter+1)*avg_time + time_elapsed)/(counter+2);
    // Uncomment the following line to print debug and log information
    // std::cout << " (Elapsed time from last in ms: " << time_elapsed << "- Average elapsed time from last in ms: " << avg_time << ")" << std::endl;
    startTime = std::chrono::high_resolution_clock::now();
    counter = counter + 1;
    
    std::string command1;
    std::string command2;
    std::string command3;

    // The following lines obtain the neuron activations to be sent to the GUI
    LOB = ltPT.GetLOB();
    LB = ltPT.GetLB();
    LIB = ltPT.GetLIB();
    LIOB = ltPT.GetLIOB();
    LIMM = ltPT.GetLIMM();
    LMM = ltPT.GetLMM();
    LOML = ltPT.GetLOML();
    LS = ltPT.GetLS();
    ROB = ltPT.GetROB();
    RB = ltPT.GetRB();
    RIB = ltPT.GetRIB();
    RIOB = ltPT.GetRIOB();
    RIMM = ltPT.GetRIMM();
    RMM = ltPT.GetRMM();
    ROML = ltPT.GetROML();
    RS = ltPT.GetRS();
    
    // Messages crafted to be sent to GUI
    command1 = "echo \"1---LMLF->" + std::to_string(LMLF) + "RMLF->" + std::to_string(RMLF) + "LLHB->" + std::to_string(LLHB) + "RLHB->" + std::to_string(RLHB) + "LOB->" + std::to_string(LOB) + "LB->" + std::to_string(LB) + "LIB->" + std::to_string(LIB) + "LIOB->" + std::to_string(LIOB) + "LIMM->" + std::to_string(LIMM) + "LMM->" + std::to_string(LMM) + "LOML->" + std::to_string(LOML) + "LS->" + std::to_string(LS) + " \" | nc " + IPv4_to_contact + " 5053 -i 0 -q 0 -w 0\n";
    system(command1.c_str());
    command2 = "echo \"2---ROB->" + std::to_string(ROB) + "RB->" + std::to_string(RB) + "RIB->" + std::to_string(RIB) + "RIOB->" + std::to_string(RIOB) + "RIMM->" + std::to_string(RIMM) + "RMM->" + std::to_string(RMM) + "ROML->" + std::to_string(ROML) + "RS->" + std::to_string(RS)  + "DSL0->" + std::to_string(smoothed_outDSGC_L_0/100) + "DSL1->" + std::to_string(smoothed_outDSGC_L_1/100) + "DSL2->" + std::to_string(smoothed_outDSGC_L_2/100) + "DSL3->" + std::to_string(smoothed_outDSGC_L_3/100) + " \" | nc " + IPv4_to_contact + " 5053 -i 0 -q 0 -w 0\n";
    execute(command2.c_str());
    command3 = "echo \"3---DSR0->" + std::to_string(smoothed_outDSGC_R_0/100) + "DSR1->" + std::to_string(smoothed_outDSGC_R_1/100) + "DSR2->" + std::to_string(smoothed_outDSGC_R_2/100) + "DSR3->" + std::to_string(smoothed_outDSGC_R_3/100) + "LVSP->" + std::to_string(LVSPNs_output) + "RVSP->" + std::to_string(RVSPNs_output) + "SSLF->" + std::to_string(SS_MLF/255
    ) + "COMM->" + std::to_string(command) + " \" | nc " + IPv4_to_contact + " 5053 -i 0 -q 0 -w 0\n";
    execute(command3.c_str());
    
    // Image acquisition + server
    // The frames for the two eyes are merged into a single frame
    // The argument '3' indicates to marge GUI frames
    cv::Mat FrameGUICurrent = MergeFrames(3);
    cvsource.PutFrame(FrameGUICurrent);
    
    // Check whether the GUI has asked to stop the execution of the program
    std::ifstream stop_from_GUI_FILE;
    stop_from_GUI_FILE.open("/home/pi/stop_from_GUI_FILE.txt");
    std::string line;
    if (stop_from_GUI_FILE.is_open())
    {
      stop_from_GUI_FILE >> line;
    }
    int stop_flag = std::stoi(line);
    
    if (stop_flag)
    {
      stop_from_GUI = 1;
    }
    stop_from_GUI_FILE.close();
  }
  while ((TimeElapsed_(globalStartTime) < ex_time_s*1000)  && (stop_from_GUI == 0));
  std::string command;
  command = "echo \"END\" | nc " + IPv4_to_contact + " 5053 -i 0 -q 0 -w 0\n";
  execute(command.c_str());
}

/******************************************************************************/

// This function initializes the robot
void Envirobot::Initialize(int GUI_FLAG)
{   

  // Open the robot's eyes
  OpenEyes(GUI_FLAG);

  // Initialize the motors
  motors_connected = OpenMotors();

  // Give the visual network time to stabilize
  std::chrono::high_resolution_clock::time_point referenceTime = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point currentTime;
  bool initializingVisualNetwork = true;
  while(initializingVisualNetwork)
  {

    // Measure the FPS realized. Get the current time stamp
    currentTime = std::chrono::high_resolution_clock::now();

    // Compute time difference between reference time stamp and current time stamp
    auto duration = std::chrono::duration_cast<std::chrono::microseconds> (currentTime - referenceTime).count();

    // If a second has passed, print the number of frames processed and reset the counter
    if (duration > NETWORK_INIT_TIME)
    {
      initializingVisualNetwork = false;
    }
  }

  /// All the neurons
  char log_file_name[255];
  char log_file_MOTORS_name[255];
  char log_file_PowerMeter_name[255];
  char log_file_TailBeatingAmplitude_name[255];
  struct tm* tm;
  time_t now;
  now = time(0); // get current time
  tm = localtime(&now); // get structure
  sprintf(log_file_name, "logFiles/log_file_%04d%02d%02d%02d%02d%02d", tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
  sprintf(log_file_MOTORS_name, "logFiles/log_file_motor_%04d%02d%02d%02d%02d%02d", tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
  sprintf(log_file_PowerMeter_name, "logFiles/log_file_power_%04d%02d%02d%02d%02d%02d", tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
  sprintf(log_file_TailBeatingAmplitude_name, "logFiles/log_file_tailAmplitude_%04d%02d%02d%02d%02d%02d", tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
  log_file = fopen(log_file_name, "w");
  log_file_MOTORS = fopen(log_file_MOTORS_name, "w");
  log_file_PowerMeter = fopen(log_file_PowerMeter_name, "w");
  log_file_TailBeatingAmplitude = fopen(log_file_TailBeatingAmplitude_name, "w");
//  log_file = fopen("log_file.txt", "w");
//  log_file_MOTORS = fopen("log_file_MOTORS.txt", "w");
  
}

// This function implements the OptoMotor Response of the fish
 void Envirobot::ExecuteOMR_and_GetTargetMotorPosition(int startup, int ex_time_s, int GUI_FLAG, std::string IPv4_to_contact)
{
  long int counter = 0;
  
  std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point globalStartTime = std::chrono::high_resolution_clock::now();
  float avg_time = 0;
  
  double outDSGC_L_0 = 0;
  double outDSGC_L_1 = 0;
  double outDSGC_L_2 = 0;
  double outDSGC_L_3 = 0;
  double outDSGC_R_0 = 0;
  double outDSGC_R_1 = 0;
  double outDSGC_R_2 = 0;
  double outDSGC_R_3 = 0;
  double LPT_R_0;
  double LPT_R_1;
  double LPT_R_2;
  double LPT_R_3;
  double RPT_L_0;
  double RPT_L_1;
  double RPT_L_2;
  double RPT_L_3;
  double LMLF_Naumann;
  double RMLF_Naumann;
  double LLHB_input;
  double RLHB_input;
  // Variables used only for demo purposes
  // double LVSPNs_input;
  // double RVSPNs_input;
  // double LVSPNs;
  // double RVSPNs;
  
  // Lines used to perform the experiments
  int counter_exp = 0;
  
  double avg_outDSGC_L_0 = 0;
  double avg_outDSGC_L_1 = 0;
  double avg_outDSGC_L_2 = 0;
  double avg_outDSGC_L_3 = 0;
  double avg_outDSGC_R_0 = 0;
  double avg_outDSGC_R_1 = 0;
  double avg_outDSGC_R_2 = 0;
  double avg_outDSGC_R_3 = 0;

  /// Direction Selective Cells in Pretectum
  double avg_LPT_R_0 = 0;
  double avg_LPT_R_1 = 0;
  double avg_LPT_R_2 = 0;
  double avg_LPT_R_3 = 0;
  double avg_RPT_L_0 = 0;
  double avg_RPT_L_1 = 0;
  double avg_RPT_L_2 = 0;
  double avg_RPT_L_3 = 0;
  /// Direction Selective Cells in late Pretectum
  double avg_LOB = 0;
  double read_LOB;
  double avg_LB = 0;
  double read_LB;
  double avg_LIB = 0;
  double read_LIB;
  double avg_LIOB = 0;
  double read_LIOB;
  double avg_LMM = 0;
  double read_LMM;
  double avg_LIMM = 0;
  double read_LIMM;
  double avg_LOML = 0;
  double read_LOML;
  double avg_LS = 0;
  double read_LS;
  double avg_ROB = 0;
  double read_ROB;
  double avg_RB = 0;
  double read_RB;
  double avg_RIB = 0;
  double read_RIB;
  double avg_RIOB = 0;
  double read_RIOB;
  double avg_RMM = 0;
  double read_RMM;
  double avg_RIMM = 0;
  double read_RIMM;
  double avg_ROML = 0;
  double read_ROML;
  double avg_RS = 0;
  double read_RS;
  /// LMLF, RMLF, LLHB, RLHB
  double avg_LMLF = 0;
  double avg_RMLF = 0;
  double avg_LLHB = 0;
  double avg_RLHB = 0;

  // Remove
  double LTT = 0;
  double RTT = 0;

  do
  {
    // Uncomment the following line to print debug and log information
    // std::cout << "ExecuteOMR_and_GetTargetMotorPosition with ID " << counter;
    float time_elapsed = TimeElapsed_(startTime);
    avg_time = ((counter+1)*avg_time + time_elapsed)/(counter+2);
    std::this_thread::sleep_for(std::chrono::milliseconds(100 - int(time_elapsed)));
    // Uncomment the following line to print debug and log information
    // std::cout << " (Elapsed time from last in ms: " << time_elapsed << "- Average elapsed time from last in ms: " << avg_time << ")" << std::endl;
    startTime = std::chrono::high_resolution_clock::now();
    
    counter = counter + 1;
    
    // The procedure is repeated for each of the eye
    for(std::vector<Eye>::iterator iter_eye = d_eyes.begin(); iter_eye != d_eyes.end(); iter_eye++)
    {
      switch (iter_eye->EyeOrientation())
      {
        case Left :
          // We obtain the unprocessed values of the Direction Selective cells in the right pretectum starting
          // from the values assumed by the Direction Selective Ganglion Cells in the Left retina
          // Left (L) Superior (0)
          outDSGC_L_0 = iter_eye->GetInputToPretectum(0);
          // Left (L) Anterior (1)
          outDSGC_L_1 = iter_eye->GetInputToPretectum(1);
          // Left (L) Inferior (2)
          outDSGC_L_2 = iter_eye->GetInputToPretectum(2);
          // Left (L) Posterior (3)
          outDSGC_L_3 = iter_eye->GetInputToPretectum(3);

          avg_outDSGC_L_0 = ((counter_exp+1)*avg_outDSGC_L_0 + outDSGC_L_0)/(counter_exp+2);
          avg_outDSGC_L_1 = ((counter_exp+1)*avg_outDSGC_L_1 + outDSGC_L_1)/(counter_exp+2);
          avg_outDSGC_L_2 = ((counter_exp+1)*avg_outDSGC_L_2 + outDSGC_L_2)/(counter_exp+2);
          avg_outDSGC_L_3 = ((counter_exp+1)*avg_outDSGC_L_3 + outDSGC_L_3)/(counter_exp+2);

	  //Automatically adjust the tranform function (omega and bias) of OFF DSGC
          iter_eye->AdjustTFOFFDSGC(outDSGC_L_0 + outDSGC_L_1 + outDSGC_L_2 + outDSGC_L_3);

//	  std::cout << "outDSGC_L_0 = " << outDSGC_L_0 << "(average = " << avg_outDSGC_L_0 << ")" << std::endl;
//	  std::cout << "outDSGC_L_1 = " << outDSGC_L_1 << "(average = " << avg_outDSGC_L_1 << ")" << std::endl;
//	  std::cout << "outDSGC_L_2 = " << outDSGC_L_2 << "(average = " << avg_outDSGC_L_2 << ")" << std::endl;
//	  std::cout << "outDSGC_L_3 = " << outDSGC_L_3 << "(average = " << avg_outDSGC_L_3 << ")" << std::endl;
        
          // We normalize the values just obtained.
          // This step is necessary to improve the robustness, as DSGCs seems very sensitive to
          // lightning conditions and to the distance at which the monitor that shows the stimulating
          // pattern is placed. Without the normalization step, the four outputs change quite rapidly,
          // but in any case the higher one is always the one associated to the motion displayed to
          // the eyes.
          // normal_max(&outDSGC_L_0, &outDSGC_L_1, &outDSGC_L_2, &outDSGC_L_3);
        


          break;
    
        case Right :
          // We obtain the unprocessed values of the Direction Selective cells in the left pretectum starting
          // from the values assumed by the Direction Selective Ganglion Cells in the Right retina
          // Right (R) Superior (0)
          outDSGC_R_0 = iter_eye->GetInputToPretectum(0);
          // Right (R) Anterior (1)
          outDSGC_R_1 = iter_eye->GetInputToPretectum(1);
          // Right (R) Inferior (2)
          outDSGC_R_2 = iter_eye->GetInputToPretectum(2);
          // Right (R) Posterior (3)
          outDSGC_R_3 = iter_eye->GetInputToPretectum(3);

          avg_outDSGC_R_0 = ((counter_exp+1)*avg_outDSGC_R_0 + outDSGC_R_0)/(counter_exp+2);
          avg_outDSGC_R_1 = ((counter_exp+1)*avg_outDSGC_R_1 + outDSGC_R_1)/(counter_exp+2);
          avg_outDSGC_R_2 = ((counter_exp+1)*avg_outDSGC_R_2 + outDSGC_R_2)/(counter_exp+2);
          avg_outDSGC_R_3 = ((counter_exp+1)*avg_outDSGC_R_3 + outDSGC_R_3)/(counter_exp+2);

	  //Automatically adjust the tranform function (omega and bias) of OFF DSGC
	  iter_eye->AdjustTFOFFDSGC(outDSGC_R_0 + outDSGC_R_1 + outDSGC_R_2 + outDSGC_R_3);

//	  std::cout << "outDSGC_R_0 = " << outDSGC_R_0 << "(average = " << avg_outDSGC_R_0 << ")" << std::endl;
//	  std::cout << "outDSGC_R_1 = " << outDSGC_R_1 << "(average = " << avg_outDSGC_R_1 << ")" << std::endl;
//	  std::cout << "outDSGC_R_2 = " << outDSGC_R_2 << "(average = " << avg_outDSGC_R_2 << ")" << std::endl;
//        std::cout << "outDSGC_R_3 = " << outDSGC_R_3 << "(average = " << avg_outDSGC_R_3 << ")" << std::endl;

          // We normalize the values just obtained.
          // normal_max(&outDSGC_R_0, &outDSGC_R_1, &outDSGC_R_2, &outDSGC_R_3);

          break;
      }
    }

    // This part implements the brain neurons involved in the OMR
if (bout_flag == 0){
    // We add a Low-Pass Filter (LPF) to smooth the unprocessed values of the Direction Selective cells in the right pretectum
    smoothed_outDSGC_L_0 = smoothed_outDSGC_L_0 * (1 - weigth_LPF_mPT) + outDSGC_L_0 * weigth_LPF_mPT;
    smoothed_outDSGC_L_1 = smoothed_outDSGC_L_1 * (1 - weigth_LPF_mPT) + outDSGC_L_1 * weigth_LPF_mPT;
    smoothed_outDSGC_L_2 = smoothed_outDSGC_L_2 * (1 - weigth_LPF_mPT) + outDSGC_L_2 * weigth_LPF_mPT;
    smoothed_outDSGC_L_3 = smoothed_outDSGC_L_3 * (1 - weigth_LPF_mPT) + outDSGC_L_3 * weigth_LPF_mPT;

    // We add a Low-Pass Filter (LPF) to smooth the unprocessed values of the Direction Selective cells in the left pretectum
    smoothed_outDSGC_R_0 = smoothed_outDSGC_R_0 * (1 - weigth_LPF_mPT) + outDSGC_R_0 * weigth_LPF_mPT;
    smoothed_outDSGC_R_1 = smoothed_outDSGC_R_1 * (1 - weigth_LPF_mPT) + outDSGC_R_1 * weigth_LPF_mPT;
    smoothed_outDSGC_R_2 = smoothed_outDSGC_R_2 * (1 - weigth_LPF_mPT) + outDSGC_R_2 * weigth_LPF_mPT;
    smoothed_outDSGC_R_3 = smoothed_outDSGC_R_3 * (1 - weigth_LPF_mPT) + outDSGC_R_3 * weigth_LPF_mPT;
}

else
	{
	float reduce_fact_in_bout = 0.2;
    // We add a Low-Pass Filter (LPF) to smooth the unprocessed values of the Direction Selective cells in the right pretectum
    smoothed_outDSGC_L_0 = smoothed_outDSGC_L_0 * (1 - weigth_LPF_mPT * reduce_fact_in_bout) + outDSGC_L_0 * weigth_LPF_mPT * reduce_fact_in_bout;
    smoothed_outDSGC_L_1 = smoothed_outDSGC_L_1 * (1 - weigth_LPF_mPT * reduce_fact_in_bout) + outDSGC_L_1 * weigth_LPF_mPT * reduce_fact_in_bout;
    smoothed_outDSGC_L_2 = smoothed_outDSGC_L_2 * (1 - weigth_LPF_mPT * reduce_fact_in_bout) + outDSGC_L_2 * weigth_LPF_mPT * reduce_fact_in_bout;
    smoothed_outDSGC_L_3 = smoothed_outDSGC_L_3 * (1 - weigth_LPF_mPT * reduce_fact_in_bout) + outDSGC_L_3 * weigth_LPF_mPT * reduce_fact_in_bout;

    // We add a Low-Pass Filter (LPF) to smooth the unprocessed values of the Direction Selective cells in the left pretectum
    smoothed_outDSGC_R_0 = smoothed_outDSGC_R_0 * (1 - weigth_LPF_mPT * reduce_fact_in_bout) + outDSGC_R_0 * weigth_LPF_mPT * reduce_fact_in_bout;
    smoothed_outDSGC_R_1 = smoothed_outDSGC_R_1 * (1 - weigth_LPF_mPT * reduce_fact_in_bout) + outDSGC_R_1 * weigth_LPF_mPT * reduce_fact_in_bout;
    smoothed_outDSGC_R_2 = smoothed_outDSGC_R_2 * (1 - weigth_LPF_mPT * reduce_fact_in_bout) + outDSGC_R_2 * weigth_LPF_mPT * reduce_fact_in_bout;
    smoothed_outDSGC_R_3 = smoothed_outDSGC_R_3 * (1 - weigth_LPF_mPT * reduce_fact_in_bout) + outDSGC_R_3 * weigth_LPF_mPT * reduce_fact_in_bout;
	}


    // Add a automatical adjust Omega and Bias to the PT DS cells
     double avg_outDSGC_L = (smoothed_outDSGC_L_0 + smoothed_outDSGC_L_1 + smoothed_outDSGC_L_2 + smoothed_outDSGC_L_3)/8;
     double avg_outDSGC_R = (smoothed_outDSGC_R_0 + smoothed_outDSGC_R_1 + smoothed_outDSGC_R_2 + smoothed_outDSGC_R_3)/8;
     //use the following omega and bias if there is not inhibition to the L/RPT_L/R_0/3 
     //omega_DSs = 0.04 - 0.01 * (avg_outDSGC_L - 200) * 0.005;
     //omega_DSs2 = 0.04 - 0.01 * (avg_outDSGC_R -200) * 0.005;
     //bias_DSs = 200 + (avg_outDSGC_L - 200) * 0.75;
     //bias_DSs2 = 200 + (avg_outDSGC_R -200) * 0.75;
     omega_DSs = 0.04 - 0.025 * (avg_outDSGC_L - 80) * 0.01;
     omega_DSs2 = 0.04 - 0.025 * (avg_outDSGC_R -80) * 0.01;
     bias_DSs = 80 + (avg_outDSGC_L - 80) * 0.4;
     bias_DSs2 = 80 + (avg_outDSGC_R -80) * 0.4;
//	std::cout << "Right eye Omega = " << omega_DSs2 << " Right eye Bias = " << bias_DSs2 << std::endl;
//	std::cout << "Left eye Omega = " << omega_DSs << " Left eye Bias = " << bias_DSs << std::endl;

    // We further process the smoothed values using a sigmoid function (we pass to the 'Sigmoid' function the omega and bias parameters)
    // and we obtain the values assumes by the Direction Selective cells in the Left Pretectum
    LPT_R_0 = Sigmoid(omega_DSs2, bias_DSs2, smoothed_outDSGC_R_0); // _R_ stands for right eye
    LPT_R_1 = Sigmoid(omega_DSs2, bias_DSs2, smoothed_outDSGC_R_1);
    LPT_R_2 = Sigmoid(omega_DSs2, bias_DSs2, smoothed_outDSGC_R_2);
    LPT_R_3 = Sigmoid(omega_DSs2, bias_DSs2, smoothed_outDSGC_R_3);

    // We further process the smoothed values using a sigmoid function (we pass to the 'Sigmoid' function the omega and bias parameters)
    // and we obtain the values assumes by the Direction Selective cells in the Right Pretectum
    RPT_L_0 = Sigmoid(omega_DSs, bias_DSs, smoothed_outDSGC_L_0); // _L_ stands for left eye
    RPT_L_1 = Sigmoid(omega_DSs, bias_DSs, smoothed_outDSGC_L_1);
    RPT_L_2 = Sigmoid(omega_DSs, bias_DSs, smoothed_outDSGC_L_2);
    RPT_L_3 = Sigmoid(omega_DSs, bias_DSs, smoothed_outDSGC_L_3);

    // We further process the smoothed values using a sigmoid function (we pass to the 'Sigmoid' function the omega and bias parameters)
    // and we obtain the values assumes by the Direction Selective cells in the Left Pretectum
    float inhibition_weight = 0.5; 
    LPT_R_0 = Sigmoid(omega_DSs2, bias_DSs2, smoothed_outDSGC_R_0 - inhibition_weight * smoothed_outDSGC_R_2); // _R_ stands for right eye
    LPT_R_1 = Sigmoid(omega_DSs2, bias_DSs2, smoothed_outDSGC_R_1 - inhibition_weight * smoothed_outDSGC_R_3);
    LPT_R_2 = Sigmoid(omega_DSs2, bias_DSs2, smoothed_outDSGC_R_2 - inhibition_weight * smoothed_outDSGC_R_0);
    LPT_R_3 = Sigmoid(omega_DSs2, bias_DSs2, smoothed_outDSGC_R_3 - inhibition_weight * smoothed_outDSGC_R_1);

    // We further process the smoothed values using a sigmoid function (we pass to the 'Sigmoid' function the omega and bias parameters)
    // and we obtain the values assumes by the Direction Selective cells in the Right Pretectum
    RPT_L_0 = Sigmoid(omega_DSs, bias_DSs, smoothed_outDSGC_L_0 - inhibition_weight * smoothed_outDSGC_L_2); // _L_ stands for left eye
    RPT_L_1 = Sigmoid(omega_DSs, bias_DSs, smoothed_outDSGC_L_1 - inhibition_weight * smoothed_outDSGC_L_3);
    RPT_L_2 = Sigmoid(omega_DSs, bias_DSs, smoothed_outDSGC_L_2 - inhibition_weight * smoothed_outDSGC_L_0);
    RPT_L_3 = Sigmoid(omega_DSs, bias_DSs, smoothed_outDSGC_L_3 - inhibition_weight * smoothed_outDSGC_L_1);    
    
    /// Average values for the experiments
    /// Direction Selective Cells in Pretectum
    avg_LPT_R_0 = ((counter_exp+1)*avg_LPT_R_0 + LPT_R_0)/(counter_exp+2);
    avg_LPT_R_1 = ((counter_exp+1)*avg_LPT_R_1 + LPT_R_1)/(counter_exp+2);
    avg_LPT_R_2 = ((counter_exp+1)*avg_LPT_R_2 + LPT_R_2)/(counter_exp+2);
    avg_LPT_R_3 = ((counter_exp+1)*avg_LPT_R_3 + LPT_R_3)/(counter_exp+2);
    avg_RPT_L_0 = ((counter_exp+1)*avg_RPT_L_0 + RPT_L_0)/(counter_exp+2);
    avg_RPT_L_1 = ((counter_exp+1)*avg_RPT_L_1 + RPT_L_1)/(counter_exp+2);
    avg_RPT_L_2 = ((counter_exp+1)*avg_RPT_L_2 + RPT_L_2)/(counter_exp+2);
    avg_RPT_L_3 = ((counter_exp+1)*avg_RPT_L_3 + RPT_L_3)/(counter_exp+2);
//    std::cout << "LPT_R_0 = " << LPT_R_0 << " (average = " << avg_LPT_R_0 << ")" << std::endl;
//    std::cout << "LPT_R_1 = " << LPT_R_1 << " (average = " << avg_LPT_R_1 << ")" << std::endl;
//    std::cout << "LPT_R_2 = " << LPT_R_2 << " (average = " << avg_LPT_R_2 << ")" << std::endl;
//    std::cout << "LPT_R_3 = " << LPT_R_3 << " (average = " << avg_LPT_R_3 << ")" << std::endl;
//    std::cout << "RPT_L_0 = " << RPT_L_0 << " (average = " << avg_RPT_L_0 << ")" << std::endl;
//    std::cout << "RPT_L_1 = " << RPT_L_1 << " (average = " << avg_RPT_L_1 << ")" << std::endl;
//    std::cout << "RPT_L_2 = " << RPT_L_2 << " (average = " << avg_RPT_L_2 << ")" << std::endl;
//    std::cout << "RPT_L_3 = " << RPT_L_3 << " (average = " << avg_RPT_L_3 << ")" << std::endl;
  
  
  /// Log files for the experiments
  /// Direction Selective Cells in Pretectum
  //log_fileDSGCR0 << LPT_R_0 << std::endl;
  //log_fileDSGCR1 << LPT_R_1 << std::endl;
  //log_fileDSGCR2 << LPT_R_2 << std::endl;
  //log_fileDSGCR3 << LPT_R_3 << std::endl;
  //log_fileDSGCL0 << RPT_L_0 << std::endl;
  //log_fileDSGCL1 << RPT_L_1 << std::endl;
  //log_fileDSGCL2 << RPT_L_2 << std::endl;
  //log_fileDSGCL3 << RPT_L_3 << std::endl;

    // We update the values of the nMLF (both Right nMLF and Left nMLF) starting from the values assumed by the Direction Selective cells
    // in the Right Pretectum and in the Left Pretectum
    ltPT.UpdateMLF(LPT_R_0, LPT_R_1, LPT_R_2, LPT_R_3, RPT_L_0, RPT_L_1, RPT_L_2, RPT_L_3);

    // We get the updated value of the Left nMLF (LMLF)
    LMLF_Naumann = ltPT.GetMLF(0);
    // We get the updated value of the Right nMLF (RMLF)
    RMLF_Naumann = ltPT.GetMLF(1);
    
    /// Average values for the experiments
    /// Direction Selective Cells in late Pretectum
    read_LOB = ltPT.GetLOB();
    read_LB = ltPT.GetLB();
    read_LIB = ltPT.GetLIB();
    read_LIOB = ltPT.GetLIOB();
    read_LMM = ltPT.GetLMM();
    read_LIMM = ltPT.GetLIMM();
    read_LOML = ltPT.GetLOML();
    read_LS = ltPT.GetLS();
    read_ROB = ltPT.GetROB();
    read_RB = ltPT.GetRB();
    read_RIB = ltPT.GetRIB();
    read_RIOB = ltPT.GetRIOB();
    read_RMM = ltPT.GetRMM();
    read_RIMM = ltPT.GetRIMM();
    read_ROML = ltPT.GetROML();
    read_RS = ltPT.GetRS();
    avg_LOB = ((counter_exp+1)*avg_LOB + read_LOB)/(counter_exp+2);
    avg_LB = ((counter_exp+1)*avg_LB + read_LB)/(counter_exp+2);
    avg_LIB = ((counter_exp+1)*avg_LIB + read_LIB)/(counter_exp+2);
    avg_LIOB = ((counter_exp+1)*avg_LIOB + read_LIOB)/(counter_exp+2);
    avg_LMM = ((counter_exp+1)*avg_LMM + read_LMM)/(counter_exp+2);
    avg_LIMM = ((counter_exp+1)*avg_LIMM + read_LIMM)/(counter_exp+2);
    avg_LOML = ((counter_exp+1)*avg_LOML + read_LOML)/(counter_exp+2);
    avg_LS = ((counter_exp+1)*avg_LS + read_LS)/(counter_exp+2);
    avg_ROB = ((counter_exp+1)*avg_ROB + read_ROB)/(counter_exp+2);
    avg_RB = ((counter_exp+1)*avg_RB + read_RB)/(counter_exp+2);
    avg_RIB = ((counter_exp+1)*avg_RIB + read_RIB)/(counter_exp+2);
    avg_RIOB = ((counter_exp+1)*avg_RIOB + read_RIOB)/(counter_exp+2);
    avg_RMM = ((counter_exp+1)*avg_RMM + read_RMM)/(counter_exp+2);
    avg_RIMM = ((counter_exp+1)*avg_RIMM + read_RIMM)/(counter_exp+2);
    avg_ROML = ((counter_exp+1)*avg_ROML + read_ROML)/(counter_exp+2);
    avg_RS = ((counter_exp+1)*avg_RS + read_RS)/(counter_exp+2);
//    std::cout << "LOB = " << read_LOB << " (average = " << avg_LOB << ")" << std::endl;
//    std::cout << "LB = " << read_LB << " (average = " << avg_LB << ")" << std::endl;
//    std::cout << "LIB = " << read_LIB << " (average = " << avg_LIB << ")" << std::endl;
//    std::cout << "LIOB = " << read_LIOB << " (average = " << avg_LIOB << ")" << std::endl;
//    std::cout << "LMM = " << read_LMM << " (average = " << avg_LMM << ")" << std::endl;
//    std::cout << "LIMM = " << read_LIMM << " (average = " << avg_LIMM << ")" << std::endl;
//    std::cout << "LOML = " << read_LOML << " (average = " << avg_LOML << ")" << std::endl;
//    std::cout << "LS = " << read_LS << " (average = " << avg_LS << ")" << std::endl;
//    std::cout << "ROB = " << read_ROB << " (average = " << avg_ROB << ")" << std::endl;
//    std::cout << "RB = " << read_RB << " (average = " << avg_RB << ")" << std::endl;
//    std::cout << "RIB = " << read_RIB << " (average = " << avg_RIB << ")" << std::endl;
//    std::cout << "RIOB = " << read_RIOB << " (average = " << avg_RIOB << ")" << std::endl;
//    std::cout << "RMM = " << read_RMM << " (average = " << avg_RMM << ")" << std::endl;
//    std::cout << "RIMM = " << read_RIMM << " (average = " << avg_RIMM << ")" << std::endl;
//    std::cout << "ROML = " << read_ROML << " (average = " << avg_ROML << ")" << std::endl;
//    std::cout << "RS = " << read_RS << " (average = " << avg_RS << ")" << std::endl;

    /// Log files for the experiments
    /// Direction Selective Cells in late Pretectum
    //log_fileLOB << read_LOB << std::endl;
    //log_fileLB << read_LB << std::endl;
    //log_fileLIB << read_LIB << std::endl;
    //log_fileLIOB << read_LIOB << std::endl;
    //log_fileLMM << read_LMM << std::endl;
    //log_fileLIMM << read_LIMM << std::endl;
    //log_fileLOML << read_LOML << std::endl;
    //log_fileLS << read_LS << std::endl;
    //log_fileROB << read_ROB << std::endl;
    //log_fileRB << read_RB << std::endl;
    //log_fileRIB << read_RIB << std::endl;
    //log_fileRIOB << read_RIOB << std::endl;
    //log_fileRMM << read_RMM << std::endl;
    //log_fileRIMM << read_RIMM << std::endl;
    //log_fileROML << read_ROML << std::endl;
    //log_fileRS << read_RS << std::endl;

    // We add a Low-Pass Filter (LPF) to smooth the values of LMLF (Left nMLF) and RMLF (Right nMLF)
    // Mutex instructions to avoid crashes
    mutex_LMLF.lock();
    LMLF = 0.9 * LMLF + 0.1 * LMLF_Naumann;
    mutex_LMLF.unlock();
    mutex_RMLF.lock();      
    RMLF = 0.9 * RMLF + 0.1 * RMLF_Naumann;
    mutex_RMLF.unlock();
  
    // We update the values of the LHB (both Right LHB and Left LHB)
    ltPT.UpdateLHB();
  
    // We get the updated value of the Left LHB (LLHB)
    LLHB_input = ltPT.GetLHB(0);
    // We get the updated value of the right LHB (RLHB)
    RLHB_input = ltPT.GetLHB(1);

    // We process the updated values of the Left LHB (LLHB) and of the Right LHB (RLHB)
    // using a sigmoid function (considering the appropriate omega and bias values) and we add
    // a Low-Pass Filter (LPF) to smooth the values of LLHB and RLHB
    // Mutex instructions to avoid crashes
    mutex_LLHB.lock();
    LLHB = 0.9 * LLHB + 0.1 * 1 / (1 + pow(E_N, -omega_LHB * (LLHB_input - bias_LHB)));
    mutex_LLHB.unlock();
    mutex_RLHB.lock();
    RLHB = 0.9 * RLHB + 0.1 * 1 / (1 + pow(E_N, -omega_LHB * (RLHB_input - bias_LHB)));
    mutex_RLHB.unlock();

    // We constain the LLHB and RLHB values to be between 0 and 1
    LLHB = clamp(LLHB, 0, 1);
    RLHB = clamp(RLHB, 0, 1);
    
    /// Average values for the experiments
    /// LMLF, RMLF, LLHB, RLHB
    avg_LMLF = ((counter_exp+1)*avg_LMLF + LMLF)/(counter_exp+2);
    avg_RMLF = ((counter_exp+1)*avg_RMLF + RMLF)/(counter_exp+2);
    avg_LLHB = ((counter_exp+1)*avg_LLHB + LLHB)/(counter_exp+2);
    avg_RLHB = ((counter_exp+1)*avg_RLHB + RLHB)/(counter_exp+2);
//    std::cout << "LMLF = " << LMLF << " (average = " << avg_LMLF << ")" << std::endl;
//    std::cout << "RMLF = " << RMLF << " (average = " << avg_RMLF << ")" << std::endl;
//    std::cout << "LLHB = " << LLHB << " (average = " << avg_LLHB << ")" << std::endl;
//    std::cout << "RLHB = " << RLHB << " (average = " << avg_RLHB << ")" << std::endl;
    
    counter_exp = counter_exp + 1;
    
    /// Log files for the experiments
    /// LMLF, RMLF, LLHB, RLHB
    //log_fileLMLF << LMLF << std::endl;
    //log_fileRMLF << RMLF << std::endl;
    //log_fileLLHB << LLHB << std::endl;
    //log_fileRLHB << RLHB << std::endl;
    
    /// Log file for the experiments
    fprintf(log_file, "%d \t \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t \t %0.2f \t % 0.2f \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t %0.2f \t %0.2f  \t \t %0.2f \t %0.2f \t %0.2f \t %0.2f \n", counter_exp, LPT_R_0, LPT_R_1, LPT_R_2, LPT_R_3, RPT_L_0, RPT_L_1, RPT_L_2, RPT_L_3, read_LOB, read_LB, read_LIB, read_LIOB, read_LIMM, read_LMM, read_LOML, read_LS, LTT, read_ROB, read_RB, read_RIB, read_RIOB, read_RIMM, read_RMM, read_ROML, read_RS, RTT, LMLF, RMLF, LLHB, RLHB);
//    std::cout << "counter_exp = " << counter_exp << std::endl;

    // This part can be calculated and displayed if it is useful for demo purposes uncommenting the following lines
    /*
    LVSPNs_input = LLHB;
    RVSPNs_input = RLHB;
    LVSPNs = 1 / (1 + pow(E_N, -omega_VSPNs * (LVSPNs_input - bias_VSPNs)));
    RVSPNs = 1 / (1 + pow(E_N, -omega_VSPNs * (RVSPNs_input - bias_VSPNs)));
    */
	//if (20 - time_elapsed > 0)
	//    std::this_thread::sleep_for(std::chrono::milliseconds(20 - int(time_elapsed)));    
  }
  while ((!startup) && (TimeElapsed_(globalStartTime) < ex_time_s*1000)  && (stop_from_GUI == 0));
}


void Envirobot::Cycle(int ex_time_s, int GUI_FLAG, std::string IPv4_to_contact)
{

  unsigned int numberOfEyes = d_eyes.size();
  
  // GUI mode (setting up the server)
  if (GUI_FLAG == 1)
  {
    cvMjpegServer.SetSource(cvsource);
  }
  
  // Functions to initialise the values of the neurons and the acquired frames
//  Envirobot::UpdateVisualInformation(&d_eyes[0], 1, ex_time_s, GUI_FLAG, IPv4_to_contact);
//  Envirobot::UpdateVisualInformation(&d_eyes[1], 1, ex_time_s, GUI_FLAG, IPv4_to_contact);
  Envirobot::ExecuteOMR_and_GetTargetMotorPosition(1, ex_time_s, GUI_FLAG, IPv4_to_contact);
//Envirobot::PowerMeterThread(1, ex_time_s, GUI_FLAG, IPv4_to_contact); 
  Envirobot::AssistanceThread(1, ex_time_s, GUI_FLAG, IPv4_to_contact);
  Envirobot::PowerMeterThread(1, ex_time_s, GUI_FLAG, IPv4_to_contact);
  if (motors_connected)
  {
    // Envirobot::MoveMotors(1, ex_time_s);
  }
  
  // Mode without GUI (4 threads are started)
  if (GUI_FLAG == 0)
  {
  //  std::thread th1(&Envirobot::UpdateVisualInformation, this, &d_eyes[0], 0, ex_time_s, GUI_FLAG, IPv4_to_contact);
  //  std::thread th2(&Envirobot::UpdateVisualInformation, this, &d_eyes[1], 0, ex_time_s, GUI_FLAG, IPv4_to_contact);
    std::thread th3(&Envirobot::ExecuteOMR_and_GetTargetMotorPosition, this, 0, ex_time_s, GUI_FLAG, IPv4_to_contact);
    std::thread th4(&Envirobot::MoveMotors, this, 0, ex_time_s);
   // std::thread th7(&Envirobot::PowerMeterThread, this, 0, ex_time_s, GUI_FLAG, IPv4_to_contact);   
    std::thread th6(&Envirobot::AssistanceThread, this, 0, ex_time_s, GUI_FLAG, IPv4_to_contact);
    std::thread th7(&Envirobot::PowerMeterThread, this, 0, ex_time_s, GUI_FLAG, IPv4_to_contact);
    //th1.join();
    //th2.join();
    th3.join();
    th4.join();
    th6.join();
    th7.join();
  }
  // GUI mode (additional thread for GUI commumication - 5 threads started)
  else if (GUI_FLAG == 1)
  {
    std::cout << "GUI MODE STARTED" << std::endl;
    std::thread th5(&Envirobot::GUICommunication, this, ex_time_s, IPv4_to_contact);
    std::thread th1(&Envirobot::UpdateVisualInformation, this, &d_eyes[0], 0, ex_time_s, GUI_FLAG, IPv4_to_contact);
    std::thread th2(&Envirobot::UpdateVisualInformation, this, &d_eyes[1], 0, ex_time_s, GUI_FLAG, IPv4_to_contact);
    std::thread th3(&Envirobot::ExecuteOMR_and_GetTargetMotorPosition, this, 0, ex_time_s, GUI_FLAG, IPv4_to_contact);
    std::thread th4(&Envirobot::MoveMotors, this, 0, ex_time_s);

    th5.join();
    th1.join();
    th2.join();
    th3.join();
    th4.join();
  }
}

// /******************************************************************************/

void Envirobot::Stop()
{

  // Close the robot's eyes
  CloseEyes();

  // Close the motors
  CloseMotors();

  
  /// Log files for the experiments
  /// Direction Selective Cells in Pretectum
  //log_fileDSGCR0.close();
  //log_fileDSGCR1.close();
  //log_fileDSGCR2.close();
  //log_fileDSGCR3.close();
  //log_fileDSGCL0.close();
  //log_fileDSGCL1.close();
  //log_fileDSGCL2.close();
  //log_fileDSGCL3.close();
  /// Direction Selective Cells in late Pretectum
  //log_fileLB.close();
  //log_fileLIB.close();
  //log_fileLIOB.close();
  //log_fileLMM.close();
  //log_fileLIMM.close();
  //log_fileLOML.close();
  //log_fileLS.close();
  //log_fileLOB.close();
  //log_fileRB.close();
  //log_fileRIB.close();
  //log_fileRIOB.close();
  //log_fileRMM.close();
  //log_fileRIMM.close();
  //log_fileROML.close();
  //log_fileRS.close();
  //log_fileROB.close();
  /// LMLF, RMLF, LLHB, RLHB
  //log_fileLMLF.close();
  //log_fileRMLF.close();
  //log_fileLLHB.close();
  //log_fileRLHB.close();
  /// All the neurons
  fclose(log_file);
  fclose(log_file_MOTORS);
  fclose(log_file_PowerMeter);
  fclose(log_file_TailBeatingAmplitude);
  behaviour_det.CloseLogFile();
std::cout << "Stop function finish " << std::endl;
}

/******************************************************************************/

// Function that merges two frames
cv::Mat Envirobot::MergeFrames(int WhichFrame)
{
  // Structures to hold the temporary and merged frames
  cv::Mat leftFrame;
  cv::Mat rightFrame;
  cv::Mat mergedFrame;

  // Get the current image from each eye
  for(std::vector<Eye>::iterator iter_eye = d_eyes.begin(); iter_eye != d_eyes.end(); iter_eye++)
  {
    switch (iter_eye->EyeOrientation())
    {
      case Left :
        leftFrame = iter_eye->CurrentFrame(WhichFrame);
        break;

      case Right :
        rightFrame = iter_eye->CurrentFrame(WhichFrame);
        break;
    }
  }

  // Concatonate the images and return
  if (leftFrame.rows == rightFrame.rows && leftFrame.cols == rightFrame.cols)
  {
    cv::hconcat(leftFrame, rightFrame, mergedFrame);
    return mergedFrame;
  }
  
  return rightFrame;
}

/******************************************************************************/

// Helper function that returns a if a is between b and c. If a is smaller than b, the
// function returns b, if a is greater than c, the function returns c
float clamp(float a, float b, float c)
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

// Helper function to determine the maximum between four float values and to normalize
// the four values. After the normalization, the largest variable assumes the value 100.0
void normal_max(double *fl_1, double *fl_2, double *fl_3, double *fl_4)
{
	if (*fl_1 >= *fl_2 && *fl_1 >= *fl_3 && *fl_1 >= *fl_4 && *fl_1 > 20)
	{
		*fl_2 = 100.0 * *fl_2 / *fl_1;
		*fl_3 = 100.0 * *fl_3 / *fl_1;
		*fl_4 = 100.0 * *fl_4 / *fl_1;
    *fl_1 = 100.0;
	}
	else if (*fl_2 >= *fl_1 && *fl_2 >= *fl_3 && *fl_2 >= *fl_4 && *fl_2 > 20)
	{
		*fl_1 = 100.0 * *fl_1 / *fl_2;
		*fl_3 = 100.0 * *fl_3 / *fl_2;
		*fl_4 = 100.0 * *fl_4 / *fl_2;
    *fl_2 = 100.0;
	}
	else if(*fl_3 >= *fl_1 && *fl_3 >= *fl_2 && *fl_3 >= *fl_4 && *fl_3 > 20)
	{
		*fl_1 = 100.0 * *fl_1 / *fl_3;
		*fl_2 = 100.0 * *fl_2 / *fl_3;
		*fl_4 = 100.0 * *fl_4 / *fl_3;
    *fl_3 = 100.0;
	}
	else if(*fl_4 >= *fl_1 && *fl_4 >= *fl_2 && *fl_4 >= *fl_3 && *fl_4 > 20)
	{
		*fl_1 = 100.0 * *fl_1 / *fl_4;
		*fl_2 = 100.0 * *fl_2 / *fl_4;
		*fl_3 = 100.0 * *fl_3 / *fl_4;
		*fl_4 = 100.0;
	}
}

// Helper function that computes the result of the sigmoid function given omega, the bias and the input value.
// The functions apply the sigmoid function (defined by the parameters omega and bias) on the input value 'in'
double Sigmoid(double omega, double bias, double in)
{
  double result = 1 / (1 + pow(E_N, -omega * (in - bias)));
  return result;
}

// Function that converts the motor position to a value suitable for the motor currently in use
std::vector<int> Envirobot::ConvertMotorPosition(std::vector<double> INPosition)
{
  std::vector<int> OUTPosition = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  
  for (int i = 0; i <=8; i++)
  {
    if (INPosition[i] >= 0)
    {
      goal_position_deg = (180 - (INPosition[i] * 180 / 3.14159));
      if (goal_position_deg < 0.1)
      {
        goal_position_deg = 0.1;
      }
      if (goal_position_deg > 179.9)
      {
        goal_position_deg = 179.9;
      }
    }
    else
    {
      INPosition[i] = - INPosition[i];
      goal_position_deg = (180 + (INPosition[i] * 180 / 3.14159));
      if (goal_position_deg < 180)
      {
        goal_position_deg = 180;
      }
      if (goal_position_deg > 359.9)
      {
        goal_position_deg = 359.9;
      }
    }
    
    // The motor is mounted upside down in the final robot
    goal_position_deg = 360 - goal_position_deg;

    OUTPosition[i] = (int) (goal_position_deg / 0.08789);
    
    // The limits are in place to account for the physical constrains in the robot
    if (OUTPosition[i] < 1200)
    {
      OUTPosition[i] = 1200;
    }
    if (OUTPosition[i] > 2900)
    {
      OUTPosition[i] = 2900;
    }
  }

  return OUTPosition;
}


// Function used to compute the tail position and the tail angle (the tail angle is calculated by forward kinematics)
void Envirobot::UpdateCoordinates()
{
  tail_position_x = 0.126 * (cos(target_motor_position[0]) + cos(target_motor_position[0] + target_motor_position[1]) + cos(target_motor_position[0] + target_motor_position[1] + target_motor_position[2]) + cos(target_motor_position[0] + target_motor_position[1] + target_motor_position[2] + target_motor_position[3]) + cos(target_motor_position[0] + target_motor_position[1] + target_motor_position[2] + target_motor_position[3] + target_motor_position[4]) + 2.43 * cos(target_motor_position[0] + target_motor_position[1] + target_motor_position[2] + target_motor_position[3] + target_motor_position[4] + target_motor_position[5]));
  tail_position_y = 0.126 * (sin(target_motor_position[0]) + sin(target_motor_position[0] + target_motor_position[1]) + sin(target_motor_position[0] + target_motor_position[1] + target_motor_position[2]) + sin(target_motor_position[0] + target_motor_position[1] + target_motor_position[2] + target_motor_position[3]) + sin(target_motor_position[0] + target_motor_position[1] + target_motor_position[2] + target_motor_position[3] + target_motor_position[4]) + 2.43 * sin(target_motor_position[0] + target_motor_position[1] + target_motor_position[2] + target_motor_position[3] + target_motor_position[4] + target_motor_position[5]));
  if (tail_position_x > 0)
  {
    tail_angle = atan(tail_position_y / tail_position_x) * 57.3;
  }
  else if (tail_position_x < 0 && tail_position_y > 0)
  {
    tail_angle = 90 - atan(tail_position_x / tail_position_y) * 57.3;
  }
  else if (tail_position_x < 0 && tail_position_y < 0)
  {
    tail_angle = -90 - atan(tail_position_x / tail_position_y) * 57.3;
  }
  else if (tail_position_x == 0 && tail_position_y > 0)
  {
    tail_angle = 90;
  }
  else if (tail_position_x == 0 && tail_position_y < 0)
  {
    tail_angle = -90;
  }
}

// Helper function that computes a random number starting from a Gaussian distribution
double rand_gaussianrand()
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

// Function used to run a command from command line, and to return the execution result in string format
// Source: https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-the-output-of-the-command-within-c-using-po
std::string execute(const char* cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe)
  {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
  {
    result += buffer.data();
  }
  return result;  
}
