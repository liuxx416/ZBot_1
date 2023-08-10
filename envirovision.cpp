#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <time.h>
#include <iostream>
#include "envirobot.hpp"
#include <string>



/******SCRIPT MACROS*******/
#define IMU 0           // Enable IMU (if CAMERA == 0, main loop will just print RPY values)
#define DEBUG 0         // Enable Debug Messages
#define IMU_CONFIG 0    // Enable IMU configuring (only needs to be run once / or over MT Manager instead)
#define PRINT_FPS 1     // Enable printing of FPS
#define HTTP_STREAM 1   // Enable Camera stream over SSH


// Helper function that computes the time that elapsed from the time point 'previousEventTime' passed as input
double TimeElapsedSince(std::chrono::high_resolution_clock::time_point previousEventTime)
{
	std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
	auto t = std::chrono::duration_cast<std::chrono::microseconds> (currentTime - previousEventTime).count();
	return t;
}

/******************************************************************************/

/************************/
/****** MAIN METHOD *****/
/************************/

int main(int argc, char *argv[]){
	
	int execution_time_seconds;
	int GUI_FLAG;
	std::string IPv4_to_contact;
	
	// Checking the number of parameters passed as command line arguments
	if (argc != 4)
	{
		std::cout << "STANDARD USAGE: './envirovision run_time_in_seconds GUI_FLAG IP_OF_GUI'" << std::endl;
		std::cout << "SETTING: run_time_in_seconds = 120 s" << std::endl;
		execution_time_seconds = 120;
		std::cout << "SETTING: GUI_FLAG = 0 (not used in GUI)" << std::endl;
		GUI_FLAG = 0;
		std::cout << "IP_OF_GUI NOT USED SINCE GUI_FLAG = 0" << std::endl;
	}
	else
	{
		execution_time_seconds = std::stoi(argv[1]);
		GUI_FLAG = std::stoi(argv[2]);

		IPv4_to_contact = argv[3];
	}	 
	
	// See https://tinyurl.com/srandREF
	srand(time(0));

	// Create a reference timestamp for measuring passes-per-second and program runtime
	std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
	std::chrono::high_resolution_clock::time_point PassRateRef = std::chrono::high_resolution_clock::now();

	// Specify the information for each connected camera.
	// It is necessary to specify the orientation of each camera to be sure
	// that the right and left eyes of the robot aren't flipped.
	std::vector<std::pair<Orientation, std::string>> cameraInfo
	{		
		// Cameras mounted on the prototype robot
//		{Right, "25794959"},
//		{Left, "48790759"}
		//{Left, "25794959"}
//{Right, "25794959"},                
//{Right, "50592559"}
                //{Left, "48790759"}
     //           {Left, "25794959"}	
};

	// Instantiate the Envirobot
	Envirobot envirobot(cameraInfo);

	// Initialize the robot
	envirobot.Initialize(GUI_FLAG);

	startTime = std::chrono::high_resolution_clock::now();
	PassRateRef = std::chrono::high_resolution_clock::now();

	// This variable must be uncommented in case the acquired frames are saved
	// int counter = 0;
	

	// Run the main processing pipline
	envirobot.Cycle(execution_time_seconds, GUI_FLAG, IPv4_to_contact);
			
		
	// If the following lines are uncommented, the acquired frames are saved.
	// This is done for debug and visualization purposes
	/*
	std::string image_name = std::string("/home/pi/Documents/envirobot_backup/envirovision/images/image_minus2_") + std::string(std::to_string(counter)) + std::string(".jpg");
	cv::imwrite(image_name, FrameMinus2);
	std::cout << "image_minus2 with sequence number " << counter << " has been saved" << std::endl;

	image_name = std::string("/home/pi/Documents/envirobot_backup/envirovision/images/image_minus1_") + std::string(std::to_string(counter)) + std::string(".jpg");
	cv::imwrite(image_name, FrameMinus1);
	std::cout << "image_minus1 with sequence number " << counter << " has been saved" << std::endl; 

	image_name = std::string("/home/pi/Documents/envirobot_backup/envirovision/images/image_current_") + std::string(std::to_string(counter)) + std::string(".jpg");
	cv::imwrite(image_name, FrameCurrent);
	std::cout << "image_current with sequence number " << counter << " has been saved" << std::endl;

	counter = counter + 1;
	*/

	std::cout << "Calling envirobot Stop method" << std::endl;
	// Stop acquiring camera images, and close the instance of the camera.
	envirobot.Stop();
	
	std::cout << "Done." << std::endl;	

	return 0;
}
