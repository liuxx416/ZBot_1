#include "eye.hpp"

#include <mutex>
extern std::mutex mutex_img_access;

// Helper function that computes and returns the time elapsed (in milliseconds) from the time_point previousEventTime
double TimeElapsed(std::chrono::high_resolution_clock::time_point previousEventTime)
{
	std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
	float t = std::chrono::duration_cast<std::chrono::microseconds> (currentTime - previousEventTime).count();
	return t/1000;
}

int GUI_FLAG = 0;


// Default constructor
Eye::Eye() :
						cam(),
						CAMERA_SERIAL_NUMBER(""),
						FOV_HEIGHT(FRAME_HEIGHT),
						FOV_WIDTH(FRAME_WIDTH),
						EYE_ORIENTATION(Left)
{  

}

/******************************************************************************/

// Custom constructor
Eye::Eye (Orientation s, std::string serialNumber) :
						cam(),
						CAMERA_SERIAL_NUMBER(serialNumber),
						FOV_HEIGHT(FRAME_HEIGHT),
						FOV_WIDTH(FRAME_WIDTH),
						EYE_ORIENTATION(s)
{

}

/******************************************************************************/

/*
	Enumerated type values for setting the camera parameters can be obtained here:
			https://github.com/jaybo/OpenCVGraph/blob/master/OpenCVGraph/Capture/CameraSDKs/Ximea/API/xiApi.h
*/

// This function initializes a camera (setting all the relevant parameters) and starts to acquire camera images
void Eye::OpenEye(int GUI)
{
	GUI_FLAG = GUI;
	
	// Open the eye's camera
	if (CAMERA_SERIAL_NUMBER.empty())
	{
		// Retrieving a handle to the first camera device available
		cam.OpenFirst();
	}
	else
	{
		// Open the camera according to its serial number
		std::vector<char> SN(CAMERA_SERIAL_NUMBER.c_str(), CAMERA_SERIAL_NUMBER.c_str() + CAMERA_SERIAL_NUMBER.size() + 1u);
		cam.OpenBySN(&SN[0]);
	}

	// Maximize the camera's bandwidth limit
	// Higher bandwidth allows better images, but also ruduces instability of communication (between camera and controller)
	//cam.SetBandwidthLimit(cam.GetBandwidthLimit_Maximum());
	cam.SetBandwidthLimit(150);

	// Set the data Format
	cam.SetImageDataFormat(XI_RGB24);

	// Set downsampling type
	cam.SetDownsamplingType(XI_SKIPPING);

	// Set downsampling
	cam.SetDownsampling(XI_DWN_7x7);

	// Minimize the sensor and image bit depth to maximize frame rate.
	cam.SetSensorDataBitDepth(cam.GetSensorDataBitDepth_Minimum());
	cam.SetImageDataBitDepth(cam.GetImageDataBitDepth_Minimum());

std::cout << "******************************************************************************"<< std::endl;


	// Specify the dimensions of the Region Of Interest (ROI)
	// If the Graphic User Interface (GUI) is used, the ROI is bigger as we want to stream the whole frame
	if (GUI_FLAG == 0)
	{
		cam.SetWidth(ROI_WIDTH);
		cam.SetHeight(ROI_HEIGHT);
	}
	else
	{
		//////cam.SetWidth(ROI_WIDTH*2);
		//////cam.SetHeight(ROI_HEIGHT*2);
                cam.SetWidth(ROI_WIDTH);
                cam.SetHeight(ROI_HEIGHT);
	}

	// Specify the offset of the Region of Interest. The left camera is upside down when installed inside the robot
	switch(EYE_ORIENTATION)
	{
		case(Right) :
		    if (GUI_FLAG == 0)
			{       // CHANGED FOR CAMERA ROTATION
				//cam.SetOffsetX(FOV_WIDTH - OFFSET_X - ROI_WIDTH);
                                cam.SetOffsetX(OFFSET_X);
			}
			else
			{
				//cam.SetOffsetX(FOV_WIDTH - OFFSET_X - ROI_WIDTH*2);
                                cam.SetOffsetX(OFFSET_X);
                                // CHANGED FOR CAMERA ROTATION
                                //cam.SetOffsetX(FOV_WIDTH - OFFSET_X - ROI_WIDTH);
			}
			cam.SetOffsetY(OFFSET_Y_RIGHT);
			break;

		case(Left) :
                        // CHANGED FOR CAMERA ROTATION
			//cam.SetOffsetX(OFFSET_X);
                        cam.SetOffsetX(FOV_WIDTH - OFFSET_X - ROI_WIDTH);
                        //////cam.SetOffsetX(FOV_WIDTH - OFFSET_X - ROI_WIDTH*2);
			cam.SetOffsetY(OFFSET_Y_LEFT);
			break;
	}

  /***************************************************************************
  *  Explaination on the setting:
  * Auto Gain: Although we tried to manually calcualte the gain, however, this method is with high computational cost.
  * Auto Gain: The orignal method provided by XIMEA is already very good
  * Auto Gain: We set the ROI to the bottom of the visual field, so that the robot can see the river bed clearly
  * AUto Exposure: We have to set the fps to 50, consequently, the exposure time should be less than 20 ms, here, we manully limited it up to 15 ms
  * Withe balance: Currently we are using the original auto withe balance function provided by XIMEA 
  ***************************************************************************/

	cam.EnableAutoExposureAutoGain();
	cam.SetAutoGainTopLimit(20);
	cam.SetAutoExposureTopLimit(15000);
	cam.SetAutoExposureAutoGainExposurePriority(0.5);
	cam.SetAutoExposureAutoGainTargetLevel(40);

	cam.SetAutoExposureAutoGainROIoffsetX(OFFSET_X);
	cam.SetAutoExposureAutoGainROIoffsetY(OFFSET_Y_LEFT);

	cam.SetAutoExposureAutoGainROIHeight(ROI_HEIGHT);
	//cam.SetAutoExposureAutoGainROIWidth(ROI_WIDTH); //not necessary
	//cam.EnableAutoExposureAutoGain();
	//cam.EnableHDR();
	cam.EnableWhiteBalanceAuto();

	// Set acquisition timing modeXI_ACQ_TIMING_MODE_FRAME_RATE
	//Three modes: XI_ACQ_TIMING_MODE_FREE_RUN, XI_ACQ_TIMING_MODE_FRAME_RATE_LIMIT
	cam.SetAcquisitionTimingMode(XI_ACQ_TIMING_MODE_FRAME_RATE);
	//cam.SetAcquisitionTimingMode(XI_ACQ_TIMING_MODE_FREE_RUN);
	
	// Set the frame rate
	cam.SetFrameRate(50);

	// Print a set of relevant camera parameters following the camera initialization
	std::cout << std::endl;
	std::cout << "\t~~~~ Camera Parameters ~~~~ "<<std::endl;
	std::cout << "Image Resolution:\t"<<cam.GetWidth() << " x " << cam.GetHeight() << std::endl;
	std::cout << "Frame Rate:\t"<<cam.GetFrameRate() <<std::endl;
	std::cout << "Sensor Clock Frequency (Hz):\t" << cam.GetSensorClockFrequencyHz() << std::endl;
	std::cout << "Sensor Data Bit Depth (bits/pixel):\t" << cam.GetSensorDataBitDepth() << std::endl;
	std::cout << "Image Data Bit Depth (bits/pixel):\t" << cam.GetImageDataBitDepth() << std::endl;
	std::cout << "Bandwidth Limit (MB/s):\t" << cam.GetBandwidthLimit() << std::endl;
	std::cout << std::endl;

	// Start acquiring camera images
	cam.StartAcquisition();
}

/******************************************************************************/

// A helper function to rotate images by a specified angle. Rotation is done in place on the image
void Rotate(cv::Mat src, double angle)
{
    cv::Point2f pt(src.cols/2., src.rows/2.);

	// Determine the transformation matrix needed to rotate the image
    cv::Mat transformationMatrix = cv::getRotationMatrix2D(pt, angle, 1.0);

	// Rotate the image using the transformation matrix
    cv::warpAffine(src, src, transformationMatrix, cv::Size(src.cols, src.rows));
}

// A helper function to obtain a matrix in which each entry corresponds to the mean of the values of the Red and Green channels
// of the corresponding pixel in the cv::Mat 'in' passed as a parameter
/// CHANGED FOR CAMERA ROTATION
//void FrameToRG(cv::Mat in, double out[FRAME_HEIGHT][FRAME_WIDTH])
void FrameToRG(cv::Mat in, double out[FRAME_WIDTH][FRAME_HEIGHT])
{
        /// CHANGED FOR CAMERA ROTATION
	//for (int i = 0; i < FRAME_HEIGHT; i++)
        for (int i = 0; i < FRAME_WIDTH; i++)
	{
                /// CHANGED FOR CAMERA ROTATION
		//for (int j = 0; j < FRAME_WIDTH; j++)
                for (int j = 0; j < FRAME_HEIGHT; j++)
		{
			// The order of the channels is B (0), G (1), R (2)
			out[i][j] = (in.at<cv::Vec3b>(i, j)[2] + in.at<cv::Vec3b>(i, j)[1])/2;
		}
	}
}

// A helper function to obtain a matrix in which each entry corresponds to the mean of the values of the Red and Green channels
// of the corresponding pixel in the cv::Mat 'in' passed as a parameter
void FrameToRG_quarter(cv::Mat in, double out[ROI_HEIGHT][ROI_WIDTH])
{	
        for (int i = 0; i < ROI_HEIGHT; i++)
	{
		for (int j = 0; j < ROI_WIDTH; j++)
		{
			// The order of the channels is B (0), G (1), R (2)
			out[i][j] = (in.at<cv::Vec3b>(i, j)[2] + in.at<cv::Vec3b>(i, j)[1])/2;
		}
	}
}

// This function obtains a single frame from the camera, records the acquisition time, processes the frame (by flipping and rotating it)
// and return it. The parameter *timerFrame_ms stores the elapsed time between the acquisition of the image and the time instant in
// which the image is returned
cv::Mat Eye::GetNewImage (float *timerFrame_ms)
{
	cv::Mat d_currentRawFrame;
	cv::Mat d_outFrame;
	cv::Mat d_outGUIFrame;
	
	d_currentRawFrame = cam.GetNextImageOcvMat();
	// Note: sperimentally, it has been observed that this time point is extremely close to the one of function start
	std::chrono::high_resolution_clock::time_point acquisitionTime = std::chrono::high_resolution_clock::now();

	// Rotate the image obtained by the camera
	switch (EYE_ORIENTATION)
	{
		case Left :	// Left eye must rotate by +90 degrees

			if (GUI_FLAG)
			{
				// The second parameter of cv::flip is set to -1 so that the raw frame is flipped around both axes
				cv::flip(d_currentRawFrame, d_outGUIFrame, -1); // Flip the image due to water effects
				/// CHANGED FOR CAMERA ROTATION
                                // cv::rotate(d_outGUIFrame, d_outGUIFrame, 0);
                                cv::rotate(d_outGUIFrame, d_outGUIFrame, 3);
				acquired_ProcessedGUIFrame = d_outGUIFrame;
				// Store the large image for GUI and remove the rest
                                /// CHANGED FOR CAMERA ROTATION
				//cv::Rect ROI(0, ROI_WIDTH, ROI_HEIGHT, ROI_WIDTH);
                                cv::Rect ROI(0, ROI_HEIGHT, ROI_WIDTH, ROI_HEIGHT);
				//////d_outFrame = d_outGUIFrame(ROI);
        			d_outFrame = d_outGUIFrame;
				//acquired_ProcessedGUIFrame = d_outFrame;
			}
			else
			{
				// The second parameter of cv::flip is set to -1 so that the raw frame is flipped around both axes
                                std::cout << "d_currentRawFrame.rows = " << d_currentRawFrame.rows << std::endl;
                                std::cout << "d_currentRawFrame.cols = " << d_currentRawFrame.cols << std::endl;
				cv::flip(d_currentRawFrame, d_outFrame, -1); // Flip the image due to water effects
				/// CHANGED FOR CAMERA ROTATION
                                // cv::rotate(d_outGUIFrame, d_outGUIFrame, 0);
                                cv::rotate(d_outFrame, d_outFrame, 3);	
                                std::cout << "d_outFrame.rows = " << d_outFrame.rows << std::endl;
                                std::cout << "d_outFrame.cols = " << d_outFrame.cols << std::endl;
			}
			break;

		case Right : // Right eye must rotate by -90 degrees

			if (GUI_FLAG)
			{
				// The second parameter of cv::flip is set to -1 so that the raw frame is flipped around both axes
				cv::flip(d_currentRawFrame, d_outGUIFrame, -1); // Flip the image due to water effects
				/// CHANGED FOR CAMERA ROTATION
				// cv::rotate(d_outGUIFrame, d_outGUIFrame, 2);
				cv::rotate(d_outGUIFrame, d_outGUIFrame, 3);
				acquired_ProcessedGUIFrame = d_outGUIFrame;
				// Store the large image for GUI and remove the rest
                                /// CHANGED FOR CAMERA ROTATION
				//cv::Rect ROI(ROI_HEIGHT, ROI_WIDTH, ROI_HEIGHT, ROI_WIDTH);
                                cv::Rect ROI(ROI_WIDTH, ROI_HEIGHT, ROI_WIDTH, ROI_HEIGHT);
				//////d_outFrame = d_outGUIFrame(ROI);
                                d_outFrame = d_outGUIFrame;
                                std::cout << "d_outFrame.rows (GUI) = " << d_outFrame.rows << std::endl;
                                std::cout << "d_outFrame.cols (GUI) = " << d_outFrame.cols << std::endl;
				//acquired_ProcessedGUIFrame = d_outFrame;
			}
			else
			{
				// The second parameter of cv::flip is set to -1 so that the raw frame is flipped around both axes
				cv::flip(d_currentRawFrame, d_outFrame, -1); // Flip the image due to water effects
				/// CHANGED FOR CAMERA ROTATION
				// cv::rotate(d_outFrame, d_outFrame, 2);
				cv::rotate(d_outFrame, d_outFrame, 3);	
			}
			break;
	}
	
	// Set the time elapsed between the acquisition of the image and the current time instant (the processing of the image takes some time)
	*timerFrame_ms = TimeElapsed(acquisitionTime);
	
	return d_outFrame;
}

// This function has the main purpose of obtaining one or more (already processed) frames
// The way in which this function operates depends on the ACQUISITION_MODE defined:
// * If the ACQUISITION_MODE is 1, the function obtains three successive frames. The time elapsed between the acquisitions is defined by
// 'acquisition_interval' (in milliseconds)
// * If the ACQUISITION_MODE is 0, the function obtains a single new frame each time is is called
void Eye::GetNewImages ()
{
	float timerFrame_ms;
	
	if (start)
	{
		globalTimeLeft = std::chrono::high_resolution_clock::now();
		globalTimeRight = std::chrono::high_resolution_clock::now();
	}
	
	// If the ACQUISITION_MODE is 1, we obtain three frames before proceeding
	// Note that, if we are in the startup phase, three frames are acquired in any case, no matter the acquisition mode
	if (ACQUISITION_MODE == 1 || start)
	{
		start = 0;
		acquired_ProcessedFrame_minus2 = GetNewImage(&timerFrame_ms);
//		std::this_thread::sleep_for(std::chrono::milliseconds(18));
		std::this_thread::sleep_for(std::chrono::milliseconds(25 - int(timerFrame_ms)));
		acquired_ProcessedFrame_minus1 = GetNewImage(&timerFrame_ms);
//		std::this_thread::sleep_for(std::chrono::milliseconds(18));
		std::this_thread::sleep_for(std::chrono::milliseconds(25 - int(timerFrame_ms)));
		acquired_ProcessedFrame = GetNewImage(&timerFrame_ms);
		
		// Mutex instructions to prevent crashing
        mutex_img_access.lock(); 
		d_currentProcessedFrame_minus2 = acquired_ProcessedFrame_minus2;
		d_currentProcessedFrame_minus1 = acquired_ProcessedFrame_minus1;
		d_currentProcessedFrame = acquired_ProcessedFrame;
		mutex_img_access.unlock();
	}
	
	// If the ACQUISITION_MODE is 0, we obtain a single frame and we update the frames so that the last three frames are stored in memory
	if (ACQUISITION_MODE == 0)
	{
		// We update the list of the last three frames as a new frame will be acquired (the frame currently stored in d_currentProcessedFrame_minus2
		// is therefore discarded)
		d_currentProcessedFrame_minus2 = d_currentProcessedFrame_minus1;
		d_currentProcessedFrame_minus1 = d_currentProcessedFrame;
		
		// We acquire a new frame and we take into account the eye orientation to obtain precisely the time elapsed between the acquisition
		// of two consecutive frames for the left eye and for the right eye (this is mainly done for debug purposes and to make sure that the time
		// elapsed is not to high)
		// Note: if the time elapsed is too high (higher than 60-70 ms), this may cause issues later on. In this case it is better to switch to
		// ACQUISITION_MODE 1, that causes a slower execution of the cycles but allows to obtain images very close in time
		switch (EYE_ORIENTATION)
	    {
			case Left :
				while (TimeElapsed(globalTimeLeft) < acquisition_interval)
				{	
				}
				d_currentProcessedFrame = GetNewImage(&timerFrame_ms);
				globalTimeLeft = std::chrono::high_resolution_clock::now();
				break;

			case Right :
				while (TimeElapsed(globalTimeRight) < acquisition_interval)
				{
				}
				d_currentProcessedFrame = GetNewImage(&timerFrame_ms);
				globalTimeRight = std::chrono::high_resolution_clock::now();
				break;
	    }
	    
	}
}

/******************************************************************************/

// This function stops the acquisition of camera images and closes the camera
void Eye::CloseEye()
{
	// Stop acquiring camera images
  cam.StopAcquisition();

	// Close the camera
  cam.Close();
  
}

/******************************************************************************/

// This function returns one of the three frames that are currently stored in memory.
// The frame that is being returned by the function depends on the input parameter 'WhichFrame'
cv::Mat Eye::CurrentFrame(int WhichFrame)
{
	switch (WhichFrame)
	{
		case 0:
			return d_currentProcessedFrame;
			break;
			
		case 1:
			return d_currentProcessedFrame_minus1;
			break;
			
		case 2:
			return d_currentProcessedFrame_minus2;
			break;
		case 3:
			return acquired_ProcessedGUIFrame;
			break;
	}
	
	return d_currentProcessedFrame;
}


/******************************************************************************/

// This function, given the type of Direction Selective Ganglion Cell (specified by the parameter typeDSGCs),
// returns the output signal of that DSGC by processing the three successive frames acquired previously.
// The types of DSGC are the following:
// 0 -> superior (upward motion with respect to the body of the fish)
// 1 -> anterior (forward motion with respect to the body of the fish)
// 2 -> inferior (downward motion with respect to the body of the fish)
// 3 -> posterior (backward motion with respect to the body of the fish)
    double Eye::GetInputToPretectum(int typeDSGCs)
{
	std::chrono::high_resolution_clock::time_point timer = std::chrono::high_resolution_clock::now();
	
	int offset_row = 0;
	int offset_col = 0;
	int flip = 0;
	double OFF_DSC_input_unfilted;
//    double OFF_BC[FRAME_HEIGHT][FRAME_WIDTH];
//    double lastOFF_BC[FRAME_HEIGHT][FRAME_WIDTH];
    double stateOFF_BC;
    double lastStateOFF_BC;
	double counter = 0;
	int difference_state;
	int difference_last_state;


	// According to which eye we are currently considering, we set which part of the frame we will consider
	// to obtain the output signal (the flip variable will be used later to make the calculations more compact)
	switch (EYE_ORIENTATION)
	{
		case Left :
			flip = 1;
			break;

		case Right :
			flip = -1;
			break;
	}

	// According to which type of DSGC we are currently analyzing, we set to which movement the
	// cell should respond by setting appropriate offsets
	switch(typeDSGCs)
	{
		case 0:
			offset_row = -GAPbi;
			offset_col = 0;
			break;
		case 1:
			offset_row = 0;
			offset_col = GAPbi;
			break;
		case 2:
                        offset_row = GAPbi;
			offset_col = 0;
			break;
		case 3:
                        offset_row = 0;
			offset_col = -GAPbi;
			break;
	}
	
	// The frames acquired previously are further processed, the resulting 'averaged' frames
	// are stored in matrices of doubles (only the Red and Green channels are considered)
	// Mutex instructions to prevent crashing are inserted
	mutex_img_access.lock();
	FrameToRG_quarter(d_currentProcessedFrame, frameCurrentRG);
	FrameToRG_quarter(d_currentProcessedFrame_minus1, frameMinus1RG);
	FrameToRG_quarter(d_currentProcessedFrame_minus2, frameMinus2RG);
	mutex_img_access.unlock();
	
    // We detect the OFF-edge of motion (modeled after the OFF Bipolar Cells) by analyzing couples of
	// consecutive frames (frameMinus1RG and frameCurrentRG, frameMinus2RG and frameMinus1RG)
	// Note: we are using a look-up table (Sigmoid_BC) to reduce the computational time (the sigmoid function is computationally expensive,
	// and as there is a huge number of bipolar cells, it is pre-calculated)
        for (int i = 0; i < ROI_HEIGHT; i++)
	{
		for (int j = 0; j < ROI_WIDTH; j++)
		{

			difference_state = frameMinus1RG[i][j] - frameCurrentRG[i][j];
			difference_last_state = frameMinus2RG[i][j] - frameMinus1RG[i][j];
			if (difference_state <= 0)
			{
				stateOFF_BC = 0;
			}
            else if (difference_state > 0)
            {
				//stateOFF_BC = Sigmoid_BC[difference_state];
				stateOFF_BC = 1 / (1 + pow(E_N, -omega_OFF_BC * (difference_state - bias_OFF_BC)));

			}
            if (difference_last_state <= 0)
            {
				lastStateOFF_BC = 0;
			}
            else if (difference_last_state > 0)
            {
				//lastStateOFF_BC = Sigmoid_BC[difference_last_state];
				lastStateOFF_BC = 1 / (1 + pow(E_N, -omega_OFF_BC * (difference_last_state - bias_OFF_BC)));
			}
  	                OFF_BC[i][j] = OFF_BC[i][j] * 0.0 + stateOFF_BC * 1;
			lastOFF_BC[i][j] = lastOFF_BC[i][j] * 0.0 + lastStateOFF_BC * 1;

		}
	}

    // Barlow and Levick detector, the temporal difference is defined by acquisition_interval
	// Only a part of the frame is used to obtain the output of the DSGC
	// Note: it has been observed that using a look-up table here does not reduce significantly the computational time,
	// introduces unwanted approximation and would need a very big look-up table. This approach has therefore been discarded
        for (int i = GAPbi; i < ROI_HEIGHT-GAP-GAPbi; i = i + GAP)
	{
      		for (int j = GAPbi; j < ROI_WIDTH-GAP-GAPbi; j = j + GAP)
		{
			OFF_DSC_input_unfilted = OFF_BC[i][j] - lastOFF_BC[i+offset_row][j+(flip*offset_col)] * weight_iBCs * 1 - lastOFF_BC[i+offset_row * 2][j+(flip*offset_col * 2)] * weight_iBCs * 0.5;

			OFF_DSC_input[i][j] = OFF_DSC_input[i][j] * 0 + OFF_DSC_input_unfilted * 1; 
			OFF_DSC[i][j] = OFF_DSC[i][j] * 0 + 1 / (1 + pow(E_N, -omega_OFF_DSC * (OFF_DSC_input[i][j] - bias_OFF_DSC))) * 1;
			counter = counter + OFF_DSC[i][j];
		}
	}
	if (typeDSGCs == 3){
//		AdjustCameraGain();
		AdjustTFBC();
		}
	//if (typeDSGCs == 0 || typeDSGCs == 2) counter = counter * 1.25;
	return counter;
	//return 0 is used to isolate the visual inputs.
	//return 0;
}

	void Eye::AdjustCameraGain()
{
	double brightnessSampleSum = 0;
	float currentGain = 0;//cam.GetGain();
	float newGain = currentGain; //Initialize the newGain to currentGain
	float currentExposureTime = 0;//cam.GetExposureTime();
	float newExposureTime = currentExposureTime; //Initialize the newExposureTime to currentExposureTime
	float lowLimitGain = 0;
	float upLimitExposureTime = 10000;
	double counterSample = 0;
	lastAdjustTime = std::chrono::high_resolution_clock::now();
	
	mutex_img_access.lock();
	for (int i = 0; i < ROI_HEIGHT; i=i+4)
	{
		for (int j = 0; j < ROI_WIDTH; j=j+4)
		{
			brightnessSampleSum = brightnessSampleSum + frameCurrentRG[i][j];
			counterSample++;
		}
	}
	mutex_img_access.unlock();
	
	newGain = currentGain - (brightnessSampleSum / counterSample - 96)/128/2;
	if (newGain > 20){newGain = 20;}
	else if (newGain < lowLimitGain){newGain = lowLimitGain;}
	else  {newGain = newGain;}
	

	switch (EYE_ORIENTATION)
	{
	case Left:
//		std::cout << "Left camera newGain = " << currentGain << " newExposureTime = " << currentExposureTime << " Brightness " << brightnessSampleSum / counterSample << std::endl;
		break;
	case Right: 
//		std::cout << "Right camera newGain = " << currentGain << " newExposureTime = " << currentExposureTime << " Brightness " << brightnessSampleSum / counterSample << std::endl;
		break;
	}
}

	void Eye::AdjustTFBC()
{
	double BCSum = 0;
	double counterSampledBC = 0;
	//double BCAverage = 0;
	
	for (int i = 0; i < ROI_HEIGHT; i=i+2)
	{
		for (int j = 0; j < ROI_WIDTH; j=j+2)
		{
			BCSum = BCSum + OFF_BC[i][j];
			counterSampledBC++;
		}
	}
	filtedBCAverage = filtedBCAverage * 0.9 + BCSum / counterSampledBC * 0.1;
	
	//Calculate the new Omega and Bias. Add a low-pass filter to cope with noise 
	omega_OFF_BC = omega_OFF_BC * 0.8 + (0.5 - 0.67 * (filtedBCAverage - 0.1)) * 0.2; //Range:  
	bias_OFF_BC = bias_OFF_BC * 0.8 + (20 + 50 * (filtedBCAverage - 0.1)) * 0.2; //Range:
	
	switch (EYE_ORIENTATION)
	{
	case Left:
//		std::cout << "Left camera: omega_OFF_BC = " << omega_OFF_BC << "bias_OFF_BC = " << bias_OFF_BC << "BCAverage = " << filtedBCAverage << std::endl;
		break;
	case Right: 
//		std::cout << "Right camera: omega_OFF_BC = " << omega_OFF_BC << "bias_OFF_BC = " << bias_OFF_BC << "BCAverage = " << filtedBCAverage << std::endl;
		break;
	}
	
}

	void Eye::AdjustTFOFFDSGC(float DSGCsum)
{
	float OFFDSGCSum = DSGCsum;
	float counterSampledOFFDSGC = 0;
	float OFFDSGCAverage = 0;


		//not a smart solution. I did this because we may change the ROI 	
        for (int i = GAPbi; i < ROI_HEIGHT-GAP-GAPbi; i = i + GAP)
	{
      		for (int j = GAPbi; j < ROI_WIDTH-GAP-GAPbi; j = j + GAP)
		{
			counterSampledOFFDSGC++;
		}
	}
	
	OFFDSGCAverage = OFFDSGCSum / (counterSampledOFFDSGC * 4);
	
	omega_OFF_DSC = omega_OFF_DSC * 0.9 + (10 - 20 * (OFFDSGCAverage - 0.1)) * 0.1;
	bias_OFF_DSC = bias_OFF_DSC * 0.9 + (0.7 + 2 * (OFFDSGCAverage - 0.1)) * 0.1;
	
	switch (EYE_ORIENTATION)
	{
	case Left:
//		std::cout << "Left camera: omega_OFF_DSC = " << omega_OFF_DSC << "bias_OFF_DSC =" << bias_OFF_DSC << "bias_OFF_DSC =" << OFFDSGCAverage << std::endl;
		break;
	case Right: 
//		std::cout << "Right camera: omega_OFF_DSC = " << omega_OFF_DSC << "bias_OFF_DSC =" << bias_OFF_DSC << "bias_OFF_DSC =" << OFFDSGCAverage << std::endl;
		break;
	}
}
