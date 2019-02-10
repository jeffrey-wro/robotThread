#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <thread> 
#include <math.h>

#include "MyRio.h"
#include "I2C.h"
#include "Motor_Controller.h"
#include "Utils.h"

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

#include "ImageSender.h"

using namespace std;
using namespace cv;

extern NiFpga_Session myrio_session;

NiFpga_Status status;

//Whether you want to receive the image
#define ENABLE_SERVER 1

void moveArm(Motor_Controller mc);
void processImg();

int main(int argc, char *argv[]) {

	//init myrio
	status = MyRio_Open();
	if (MyRio_IsNotSuccess(status))
	{
		return status;
	}

	//setup i2c communication  with motor controller
	MyRio_I2c i2c;
	status = Utils::setupI2CB(&myrio_session, &i2c);

	Motor_Controller mc = Motor_Controller(&i2c);
	mc.controllerEnable(DC);
	mc.controllerEnable(SERVO);

	//quick test just printing the battery volt
	int volt = mc.readBatteryVoltage(1);
	printf("%d\n\n", volt);


	thread moveArmThread (moveArm, mc);     // spawn new thread that calls foo()
	thread processImgThread (processImg);  // spawn new thread that calls bar(0)

	cout << "main, moveArm and processImg now execute concurrently..." << endl;

	//synchronize threads:
	moveArmThread.join();
	processImgThread.join();

	cout << "moveArm and processImg completed." << endl;

	//Clean  up
	mc.controllerReset(DC);
	mc.controllerReset(SERVO);

	status = MyRio_Close();

	return status;

	return 0;
}

void moveArm(Motor_Controller mc) 
{
	//move to arm up and down 2 times
	for(int i=0; i<10; i++){
		mc.setCRServoState(SERVO, CR_SERVO_1, 100);	
		Utils::waitFor(2);
		mc.setCRServoState(SERVO, CR_SERVO_1, -100);
		Utils::waitFor(2);
	}
	mc.setCRServoState(SERVO, CR_SERVO_1, 0);
	Utils::waitFor(2);

	//Clean  up
	mc.controllerReset(DC);
	mc.controllerReset(SERVO);

}

void processImg()
{    

//init image server
#if  ENABLE_SERVER
	ImageSender imageSender;

	if(imageSender.init() < 0)
	{
		return;
	}
#endif

	//open camera
	VideoCapture cap(0);
	if(!cap.isOpened())
		return;

	cap.set(CAP_PROP_FRAME_WIDTH,320);
	cap.set(CAP_PROP_FRAME_HEIGHT,200);

	Mat edges; //the processed image
    clock_t current_ticks, delta_ticks; //used to calculate fps
    clock_t fps = 0; //current fps
    for(;;)
    {
        current_ticks = clock();

        Mat frame; // the captured image
        cap >> frame; // get a new image from camera

        //process the image
        cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);

    	//send the image
	#if  ENABLE_SERVER
	    imageSender.send(&edges);
	#endif

	    //calculate fps
        delta_ticks = clock() - current_ticks;
        if(delta_ticks > 0)
            fps = CLOCKS_PER_SEC / delta_ticks;
        cout << fps << endl;
	}
}