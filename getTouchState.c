/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:
  
  QueryDevice.c

Description:

  This example demonstrates how to retrieve information from the haptic device.

*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
# include "conio.h"
# include <string.h>
#endif

#include <stdio.h>
#include <assert.h>

#include <HD/hd.h>

#include <HDU/hduVector.h>
#include <HDU/hduError.h>

/* Holds data retrieved from HDAPI. */
typedef struct 
{
    HDboolean m_buttonState1;       /* Has the device button has been pressed. */
	HDboolean m_buttonState2;
	hduVector3Dd m_devicePosition; /* Current device coordinates. */
	hduVector3Dd m_gimbalAngle; /* Current gimbal angels*/
	hduVector3Dd m_jointAngle; /* Current joint angels*/
	double m_frame[16];
    HDErrorInfo m_error;
} DeviceData;

static DeviceData gServoDeviceData;

/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData)
{   
    int nButtons = 0;

    hdBeginFrame(hdGetCurrentDevice());

    /* Retrieve the current button(s). */
    //hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    
    /* In order to get the specific button 1 state, we use a bitmask to
       test for the HD_DEVICE_BUTTON_1 bit. */
    gServoDeviceData.m_buttonState1 = 
        (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;

	/* In order to get the specific button 1 state, we use a bitmask to
	test for the HD_DEVICE_BUTTON_1 bit. */
	gServoDeviceData.m_buttonState2 =
		(nButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;

    /* Get the current location of the device (HD_GET_CURRENT_POSITION)
       We declare a vector of three doubles since hdGetDoublev returns 
       the information in a vector of size 3. */
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);

	/* Get the current location of the device (HD_CURRENT_GIMBAL_ANGLES)
	We declare a vector of three doubles since hdGetDoublev returns
	the information in a vector of size 3. */
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gServoDeviceData.m_gimbalAngle);

	/* Get the current location of the device (HD_CURRENT_GIMBAL_ANGLES)
	We declare a vector of three doubles since hdGetDoublev returns
	the information in a vector of size 3. */
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, gServoDeviceData.m_jointAngle);

	hdGetDoublev(HD_CURRENT_TRANSFORM, gServoDeviceData.m_frame);

    /* Also check the error state of HDAPI. */
    gServoDeviceData.m_error = hdGetError();

    /* Copy the position into our device_data tructure. */
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_CONTINUE;    
}


/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData)
{
    DeviceData *pDeviceData = (DeviceData *) pUserData;

    memcpy(pDeviceData, &gServoDeviceData, sizeof(DeviceData));

    return HD_CALLBACK_DONE;
}


/*******************************************************************************
 Prints out a help string about using this example.
*******************************************************************************/
void printHelp(void)
{
    static const char help[] = {"\
Press and release the stylus button to print out the current device location.\n\
Press and hold the stylus button to exit the application\n"};

    fprintf(stdout, "%s\n", help);
}


/*******************************************************************************
 This routine allows the device to provide information about the current 
 location of the stylus, and contains a mechanism for terminating the 
 application.  
 Pressing the button causes the application to display the current location
 of the device.  
 Holding the button down for N iterations causes the application to exit. 
*******************************************************************************/
void mainLoop(void)
{
    static const int kTerminateCount = 1000;
    int buttonHoldCount = 0;
	int keypress;

    /* Instantiate the structure used to capture data from the device. */
    DeviceData currentData;
    DeviceData prevData;

    /* Perform a synchronous call to copy the most current device state. */
    hdScheduleSynchronous(copyDeviceDataCallback, 
        &currentData, HD_MIN_SCHEDULER_PRIORITY);

    memcpy(&prevData, &currentData, sizeof(DeviceData));    

    printHelp();
	fflush(stdout);
    /* Run the main loop until the gimbal button is held. */
    while (1)
    {
		keypress = getchar();
		keypress = toupper(keypress);

		if (keypress == 'Q')
		{
			break;
		}
		else {

			/* Perform a synchronous call to copy the most current device state.
			   This synchronous scheduler call ensures that the device state
			   is obtained in a thread-safe manner. */
			hdScheduleSynchronous(copyDeviceDataCallback,
				&currentData,
				HD_MIN_SCHEDULER_PRIORITY);

			// Print position then angle, each vector of 3	
			/*fprintf(stdout, "%g %g %g %g %g %g %g %g %g %d\n",
				currentData.m_devicePosition[0],
				currentData.m_devicePosition[1],
				currentData.m_devicePosition[2],
				currentData.m_jointAngle[0],
				currentData.m_jointAngle[1],
				currentData.m_jointAngle[2],
				currentData.m_gimbalAngle[0],
				currentData.m_gimbalAngle[1],
				currentData.m_gimbalAngle[2],
				currentData.m_buttonState1);*/
				
			//fprintf(stdout, "%g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g \n",
			fprintf(stdout, "%g %g %g %g %g %g %g %g %g\n",
				currentData.m_frame[0],
				currentData.m_frame[1],
				currentData.m_frame[2], 
				//currentData.m_frame[3], 
				currentData.m_frame[4], 
				currentData.m_frame[5],
				currentData.m_frame[6],
				//currentData.m_frame[7],
				currentData.m_frame[8],
				currentData.m_frame[9],
				currentData.m_frame[10]
				//currentData.m_frame[11],
				//currentData.m_frame[12],
				//currentData.m_frame[13],
				//currentData.m_frame[14],
				//currentData.m_frame[15]
				);

			//hduMatrix rotMatrix(currentData.m_frame);
			//rotMatrix.getRotationMatrix(rotMatrix); 
			//hduQuaternion q = hduQuaternion(rotMatrix);

			//hduVector3Dd current_axis;
			//double current_angle;
			//q.toAxisAngle(current_axis, current_angle)
			//	fprintf(stdout, "%g %g %g %g\n",
			//	current_axis[0],
			//	current_axis[1],
			//	current_axis[2],
			//	current_angle);

			fflush(stdout);
			/* Check if an error occurred. */
			if (HD_DEVICE_ERROR(currentData.m_error))
			{
				hduPrintError(stderr, &currentData.m_error, "Device error detected");

				if (hduIsSchedulerError(&currentData.m_error))
				{
					/* Quit, since communication with the device was disrupted. */
					fprintf(stderr, "\nPress any key to quit.\n");
					fflush(stderr); 
					getch();
					break;
				}
			}
		}
    }
}

/*******************************************************************************
 Main function.
 Sets up the device, runs main application loop, cleans up when finished.
*******************************************************************************/
int main(int argc, char* argv[])
{
    HDSchedulerHandle hUpdateHandle = 0;
    HDErrorInfo error;

    /* Initialize the device, must be done before attempting to call any hd 
       functions. */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize the device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }

    /* Schedule the main scheduler callback that updates the device state. */
    hUpdateHandle = hdScheduleAsynchronous(
        updateDeviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    /* Start the servo loop scheduler. */
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }
    
    /* Run the application loop. */
    mainLoop();

    /* For cleanup, unschedule callbacks and stop the servo loop. */
    hdStopScheduler();
    hdUnschedule(hUpdateHandle);
    hdDisableDevice(hHD);

    return 0;
}

/******************************************************************************/
