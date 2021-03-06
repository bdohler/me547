/*
Copyright (c) 2013, Siddhant Ahuja (Sid)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of the <organization> nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This program contains the source code for controlling the CRS A255 arm

*/
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <algorithm>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include <stdint.h>
#include <math.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// includes for computer vision
// #include <Windows.h>
#include "opencv2/opencv.hpp"
// #include "opencv2\highgui.hpp"
// #include "opencv2\imgproc.hpp"
// #include "opencv2/imgcodecs/imgcodecs.hpp"
// #include "opencv2/videoio/videoio.hpp"
// # include "opencv2/core/version.hpp"

#include "robotix.h"

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

//added namespace for computer vision
using namespace cv;
using namespace std;
using namespace boost;




class serialConnectA255
{
private:
	//Private Variable Declaration	
	double currX;
	double currY;
	double currZ;
	double currXRot;
	double currYRot;
	double currZRot;

	double motor_1;
	double motor_2;
	double motor_3;
	double motor_4;
	double motor_5;

	double curTheta_1;
	double curTheta_2;
	double curTheta_3;
	double curTheta_4;
	double curTheta_5;

	//Initialize NodeHandle
	ros::NodeHandle n_;

	//Initialize ROS Publisher


	//Initialize ROS Subscriber


	//Declare private variables
	std::string serial_port_; //Serial Port name
	int baud_rate_; //Baud Rate for Serial Port
	int fd_; //Serial Port file descriptor
	int error_count_;

public:
	//Public Variable Declaration

	//Constructor for the Class
	serialConnectA255()
	{
		//Initialize variables
		fd_ = -1;


		//Advertise publishable topics


		//Subscribe to a topic

		//Create an internal node handler
		ros::NodeHandle n_private("serial");

		//Grab the default values from the parameter server
		n_private.param("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
		// Baud Rate
		n_private.param("baud_rate", baud_rate_, 57600); // Baud Rate

		// Build CV subscriber
		
	}


	//De-constructor for the Class.
	~serialConnectA255()
	{
		ROS_INFO("De-constructor Called");
		closePort();
	}

	//Declare all Public functions

	int openPort();

	int closePort();

	int initialize();

	int writeString(string str);

	int readString(string &str);

	void sleepms(int milliseconds);

	int getArmPositionRW();

	int getAngles();

	int goReady();

	int grip_open();
		
	int grip_close();

	double* forwKine(double desTh_1, double desTh_2, double desTh_3, double desTh_4, double desTh_5);

	double* invKine(double desX, double desY, double desZ, double alpha);

	int goInZ(double desZ);

	// Function to move the robot to a desired position in the real world
	int moveRobot(double x, double y, double z, double z_rot, double y_rot, double x_rot);

	// Function to move each joint to its desired angle
	int moveTheta(double desSpeed, double theta_1, double theta_2, double theta_3, double theta_4, double theta_5);

	//void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmdVelMsg);

	static void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};
serialConnectA255* serial;


void serialConnectA255::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("Got Image");
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

//Open the Serial Port
int serialConnectA255::openPort()
{
	//Check if serial port is already open. If so, close it.
	closePort();

	ROS_INFO("Opening CRS A255 Controller port");


	//Open the port
	/*
	1. O_RDWR : Open for reading and writing
	2. O_SYNC : Write I/O operations on the file descriptor complete as defined by synchronised I/O file integrity completion.
	3. O_NONBLOCK : If O_NONBLOCK is set, then open() function will return without blocking for the device to be ready or available. If O_NONBLOCK is clear, then the open() function will block the calling thread until the device is ready or available before returning.
	4. O_NOCTTY : If set and path identifies a terminal device, open() will not cause the terminal device to become the controlling terminal for the process.
	5. S_IRUSR : read permission, owner
	6. S_IWUSR : write permission, owner
	*/
	fd_ = open(serial_port_.c_str(), O_RDWR | O_SYNC | O_NONBLOCK | O_NOCTTY, S_IRUSR | S_IWUSR);

	//Check if opening the port was successfull
	if (fd_ == -1)
	{
		switch (errno)
		{
		case EACCES:
			ROS_ERROR("You probably don't have premission to open the port for reading and writing.");
			return -1;
			break;
		case ENOENT:
			ROS_ERROR("The requested port does not exist. Is the IMU connected? Was the port name misspelled?");
			return -1;
			break;
		}

		ROS_ERROR("serialConnectA255::Exception, Unable to open serial port [%s]. %s", serial_port_.c_str(), strerror(errno));
	}

	//Try to lock the port
	struct flock fl;
	fl.l_type = F_WRLCK; //Exclusive write lock
	fl.l_whence = SEEK_SET; //lock the whole file
	fl.l_start = 0; //Start of the blocking lock
	fl.l_len = 0; //Length of the blocking lock
	fl.l_pid = getpid(); //Process ID of the process holding the blocking lock

	//Check if the lock was successfull
	if (fcntl(fd_, F_SETLK, &fl) != 0)
	{
		ROS_ERROR("serialConnectA255::Exception, Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", serial_port_.c_str(), serial_port_.c_str());
		return -1;
	}

	//Try to change port settings
	struct termios term;

	//Try and get port attributes
	if (tcgetattr(fd_, &term) < 0)
	{
		ROS_ERROR("serialConnectA255::Exception, Unable to get serial port attributes. The port you specified (%s) may not be a serial port.", serial_port_.c_str());
		return -1;
	}

	cfmakeraw(&term); /*Sets the terminal to something like the "raw" mode of the old Version 7 terminal driver: input is available character by character, echoing is disabled, and all special processing of terminal input and output characters is disabled. The terminal attributes are set as follows:
					  termios_p->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
					  | INLCR | IGNCR | ICRNL | IXON);
					  termios_p->c_oflag &= ~OPOST;
					  termios_p->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
					  termios_p->c_cflag &= ~(CSIZE | PARENB);
					  termios_p->c_cflag |= CS8;
					  */
	cfsetispeed(&term, B57600); //Set input baud rate
	cfsetospeed(&term, B57600); //Set output baud rate

	//Set the parameters associated with the terminal 
	if (tcsetattr(fd_, TCSAFLUSH, &term) < 0)
	{
		ROS_ERROR("serialConnectA255::Exception, Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", serial_port_.c_str()); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.
		return -1;
	}

	// Make sure queues are empty before we begin
	if (tcflush(fd_, TCIOFLUSH) != 0)
	{
		ROS_ERROR("serialConnectA255::Exception, Tcflush failed. Please report this error if you see it.");
		return -1;
	}

	return 0;
}

//Close the Serial Port
int serialConnectA255::closePort()
{
	ROS_INFO("Closing Port");
	//Check if the port is already open or not
	if (fd_ != -1)
	{
		// Exit ash test
		ROS_INFO("Exiting 'ash test'");
		int status;
		status = writeString("exit\r");
		if (status != 0)
		{
			ROS_ERROR("Unable to send 'exit'");
			return -1;
		}
		sleepms(100);
		status = writeString("y\r");

		if (status != 0)
		{
			ROS_ERROR("Unable to send 'y'");
			return -1;
		}
		sleepms(100);
		if (close(fd_) != 0)
		{
			ROS_ERROR("Unable to close serial port; [%s]", strerror(errno));
			return -1;
		}
		fd_ = -1;
		return 0;
	}
}

int serialConnectA255::initialize()
{
	try
	{
		string read;

		// Get into "ash test"

		int status;
		ROS_INFO("Entering 'ash test'");

		status = writeString("ash test\r");
		if (status != 0)
		{
			ROS_ERROR("Unable to enter 'ash test' mode.");
			return -1;
		}

		sleepms(1000);
		// Drain command data from buffer
		status = readString(read);
		sleepms(100);
		status = readString(read);
		sleepms(100);

		// Get the version information for application shell
		ROS_INFO("Reading Robot Version information");
		status = writeString("robotver\r");
		sleepms(1000);

		// Try and read the string
		status = readString(read);
		if (status != 0)
		{
			ROS_ERROR("Unable to read 'robotver'");
			return -1;
		}
		read = read.substr(27, read.length() - 36);
		std::cout << read << std::endl;
		sleepms(1000);

		// Find Current Position
		getArmPositionRW();
		sleepms(1000);
		//go to Ready position
		goReady();

		return 0;

	}
	catch (std::exception &e)
	{
		error_count_++;
		ROS_ERROR("Caught an exception of an unexpected type: %s", e.what());
		return -1;
	}
	catch (...)
	{
		ROS_ERROR("Caught an unknown exception.");
		return -1;
	}
}

int serialConnectA255::getArmPositionRW()
{
	ROS_INFO("Getting Robot Arm Current Position");

	// Flush read buffer
	int status;
	string read;
	status = readString(read);

	status = writeString("here\r");
	if (status != 0)
	{
		ROS_ERROR("Unable to send 'here'");
		return -1;
	}
	sleepms(100);

	status = readString(read);
	if (status != 0)
	{
		ROS_ERROR("Unable to read 'here'");
		return -1;
	}
	//std::cout << read << std::endl;
	//std::cout << "read length : " << read.length() << std::endl;
	read = read.substr(169, read.length());
	//std::cout << read << std::endl;

	//std::cout << read << std::endl;
	vector <string> fields;
	split(fields, read, is_any_of("\n"));
	read = fields[0];
	trim(read);
	//std::cout << read << std::endl;
	//Split again on the space
	split(fields, read, is_any_of(" "));

	int i = 0;
	//cout << "fields size: " << fields.size() << "\n";
	for (size_t n = 0; n < fields.size(); n++)
	{
		if (!iequals(fields[n], ""))
		{
			switch (i)
			{
			case 0:
				currX = lexical_cast<double>(fields[n]);
				//cout << "x : " << currX << "\n";
				i++;
				break;
			case 1:
				currY = lexical_cast<double>(fields[n]);
				//cout << "y : " << currY << "\n";
				i++;
				break;
			case 2:
				currZ = lexical_cast<double>(fields[n]);
				//cout << "z : " << currZ << "\n";
				i++;
				break;
			case 3:
				currZRot = lexical_cast<double>(fields[n]);
				//cout << "zRot : " << currZRot << "\n";
				i++;
				break;
			case 4:
				currYRot = lexical_cast<double>(fields[n]);
				//cout << "yRot : " << currYRot << "\n";
				i++;
				break;
			case 5:
				currXRot = lexical_cast<double>(fields[n]);
				//cout << "xRot : " << currXRot << "\n";
				i++;
				break;
			default:
				break;
			}

		}
		//cout << "\"" << fields[ n ] << "\"\n";
	}
	return 0;
}

string setPrecision(double input, int prec)
{
	stringstream buff;
	string temp;
	buff << setprecision(prec) << setiosflags(ios_base::fixed) << input; // decimal precision
	buff >> temp;
	return temp;
}

// Function to get the joint angles (in motor counts) and convert to angle values in CRS convention:
int serialConnectA255::getAngles()
{
	ROS_INFO("Reading Joint Angles");

	// Flush read buffer
	int status;
	string readAng;
	status = readString(readAng);

	status = writeString("w2\r");
	if (status != 0)
	{
		ROS_ERROR("Unable to send 'w2'");
		return -1;
	}
	sleepms(100);

	status = readString(readAng);
	if (status != 0)
	{
		ROS_ERROR("Unable to read 'w2'");
		return -1;
	}
	sleepms(200);

	//This line to be changed...
	readAng = readAng.substr(106, readAng.length());
	sleepms(200);

	vector <string> fieldsAng;
	split(fieldsAng, readAng, is_any_of("\n"));
	readAng = fieldsAng[0];
	trim(readAng);

	//Split again on the space
	split(fieldsAng, readAng, is_any_of(" "));
	cout << readAng << "\n";

	int i = 0;
	for (size_t n = 0; n < fieldsAng.size() - 1; n++)
	{
		if (!iequals(fieldsAng[n], ""))
		{
			switch (i)
			{
			case 0:
				motor_1 = lexical_cast<double>(fieldsAng[n]);
				curTheta_1 = motor_1 / 200.0;
				cout << "curTheta_1: " << curTheta_1 << "\n";
				sleepms(200);
				i++;
				break;
			case 1:
				motor_2 = lexical_cast<double>(fieldsAng[n]);
				curTheta_2 = -motor_2 / 200.0;
				cout << "curTheta_2: " << curTheta_2 << "\n";
				sleepms(200);
				i++;
				break;
			case 2:
				motor_3 = lexical_cast<double>(fieldsAng[n]);
				curTheta_3 = motor_3 / 200.0;
				cout << "curTheta_3: " << curTheta_3 << "\n";
				sleepms(200);
				i++;
				break;
			case 3:
				motor_4 = lexical_cast<double>(fieldsAng[n]);
				curTheta_4 = -motor_4 / 44.5;
				cout << "curTheta_4: " << curTheta_4 << "\n";
				sleepms(200);
				i++;
				break;
			case 4:
				motor_5 = lexical_cast<double>(fieldsAng[n]);
				curTheta_5 = (motor_4 + motor_5) / 22.2;
				cout << "curTheta_5: " << curTheta_5 << "\n";
				sleepms(200);
				i++;
				break;
			default:
				break;
			}

		}
	}

	return 0;
}

//Forward Kinematics
/*
double* serialConnectA255::forwKine(double desTh_1, double desTh_2, double desTh_3, double desTh_4, double desTh_5)
{
ROS_INFO("Solving Forward Kinematics");

double xFound;

//Convert degrees to radians
const double PI = 22/7;
desTh_1 = desTh_1*PI/180;
desTh_2 = desTh_2*PI/180;
desTh_3 = desTh_3*PI/180;
desTh_4 = desTh_4*PI/180;
desTh_5 = desTh_5*PI/180;
sleepms(500);

double *forwK=new double[3];

//Fill in the following:
forwK[0] = ;
forwK[1] = ;
forwK[2] = ;

//Print
cout << "xDes : " << forwK[0] << "\n";
cout << "yDes : " << forwK[1] << "\n";
cout << "zDes : " << forwK[2] << "\n";

return forwK;
}
*/

//Inverse Kinematics
//desX, desY, desZ are in millimeters, alpha in degrees
double* serialConnectA255::invKine(double desX, double desY, double desZ, double alpha)
{
	ROS_INFO("Solving Inverse Kinematics");

	//Declare Variables
	const double pi = 22/7;
	cout << "Pi Calculation, is IT a INTEGER?!? " <<pi << "\n";
	alpha = alpha*pi/180;
	

	double r = sqrt(desX*desX+desY*desY);
	double s = (desZ-254) ;   // subtract d_1 from the height of the end-effector
	double A2 = 254.0;
	double A3 = 254.0; 
	double F = (pow(desX,2)+pow(desY,2)+pow(s,2)-pow(A2,2)-pow(A3,2))/(2*A2*A3); // cos(theta_3).
	
	
	//double F = 1;

	sleepms(200);

	double theta_1 = atan2(desY,desX);  // There is no offset "d".
	double theta_3 = atan2(-1*sqrt(1-pow(F,2)),F);  // theta_3 with respect to theLink 2 frame.
	double theta_2 = atan2(s,r) - atan2(A3*sin(theta_3), A2+A3*cos(theta_3));
	sleepms(200);

	double theta_4 = -theta_3-theta_2-pi/2;
	double theta_5 = alpha; //theta_1-alpha-pi/2 ;
	sleepms(200);

	double *invK=new double[5];

	//Convert from DH to CRS
	invK[0] = theta_1*180/pi;
	invK[1] = theta_2*180/pi;
	invK[2] = (theta_3+theta_2)*180/pi;
	invK[3] = (theta_4+theta_3+theta_2)*180/pi;
	invK[4] = (theta_1+theta_5)*180/pi;

	//Print
	cout << "DesTh_1 : " << invK[0] << "\n";
	cout << "DesTh_2 : " << invK[1] << "\n";
	cout << "DesTh_3 : " << invK[2] << "\n";
	cout << "DesTh_4 : " << invK[3] << "\n";
	cout << "DesTh_5 : " << invK[4] << "\n";
	sleepms(500);
	
	return invK;
}

//moveTheta function
int serialConnectA255::moveTheta(double desSpeed, double finalTheta_1, double finalTheta_2, double finalTheta_3, double finalTheta_4, double finalTheta_5)
{
	// Get Current Arm Position
	getAngles();

	// Set the joint speed
	ostringstream spConverted;
	spConverted << desSpeed;  // convert from int to str
	string tempSp = "";
	tempSp = "speed";
	tempSp = tempSp + " " + spConverted.str() + "\r";
	cout << tempSp << "\n";

	int status = writeString(tempSp);
	if (status != 0)
	{
		ROS_ERROR("Unable to send desired speed command");
		return -1;
	}
	sleepms(200);
	tempSp = "";


	// Calculate Offsets between current and desired angles
	double offsetTheta_1 = 0.0;
	double offsetTheta_2 = 0.0;
	double offsetTheta_3 = 0.0;
	double offsetTheta_4 = 0.0;
	double offsetTheta_5 = 0.0;

	offsetTheta_1 = finalTheta_1 - curTheta_1;
	offsetTheta_2 = finalTheta_2 - curTheta_2;
	offsetTheta_3 = finalTheta_3 - curTheta_3;
	offsetTheta_4 = finalTheta_4 - curTheta_4;
	offsetTheta_5 = finalTheta_5 - curTheta_5;


	string tempStr = "";
	int jointNo = 0;
	int kk = 1;
	for (kk; kk < 6; kk++)
	{
		jointNo = kk;
		ostringstream kkConverted;
		kkConverted << kk;  // convert from int to str
		tempStr = "joint";
		ostringstream deg1Converted;
		ostringstream deg2Converted;
		ostringstream deg3Converted;
		ostringstream deg4Converted;
		ostringstream deg5Converted;
		switch (jointNo)
		{
		case 1:
			deg1Converted << offsetTheta_1;  // convert from int to str
			tempStr = tempStr + " " + kkConverted.str() + "," + deg1Converted.str() + "\r";
			break;
		case 2:
			deg2Converted << offsetTheta_2;  // convert from int to str
			tempStr = tempStr + " " + kkConverted.str() + "," + deg2Converted.str() + "\r";
			break;
		case 3:
			deg3Converted << offsetTheta_3;  // convert from int to str
			tempStr = tempStr + " " + kkConverted.str() + "," + deg3Converted.str() + "\r";
			break;
		case 4:
			deg4Converted << offsetTheta_4;  // convert from int to str
			tempStr = tempStr + " " + kkConverted.str() + "," + deg4Converted.str() + "\r";
			break;
		case 5:
			deg5Converted << offsetTheta_5;  // convert from int to str
			tempStr = tempStr + " " + kkConverted.str() + "," + deg5Converted.str() + "\r";
			break;
		default:
			break;
		}
		// Send the joint command
		cout << tempStr << "\n";
		status = writeString(tempStr);
		if (status != 0)
		{
			ROS_ERROR("Unable to send joint command");
			return -1;
		}
		sleepms(8000);
		tempStr = "";
	}
	// Get Current Arm Position
	getAngles();
}

int serialConnectA255::moveRobot(double x, double y, double z, double z_rot, double y_rot, double x_rot)
{
	// Get Current Arm Position
	getArmPositionRW();
	ROS_INFO("Got Arm Positon");
	// Calculate Offsets between current and desired
	double offsetX = 0.0;
	double offsetY = 0.0;
	double offsetZ = 0.0;
	double offsetZRot = 0.0;
	double offsetYRot = 0.0;
	double offsetXRot = 0.0;

	offsetX = x - currX;
	offsetY = y - currY;
	offsetZ = z - currZ;
	offsetZRot = z_rot - currZRot;
	offsetYRot = y_rot - currYRot;
	offsetXRot = x_rot - currXRot;

	// Declare a dummy variable called dummypoint
	int status = writeString("here dummypoint\r");
	if (status != 0)
	{
		ROS_ERROR("Unable to send 'here dummypoint'");
		return -1;
	}
	sleepms(100);

	// Send a wshift command with offsets
	string wShift;
	wShift = "wshift dummypoint,";
	wShift = wShift + setPrecision(offsetX, 3) + "," + setPrecision(offsetY, 3) + "," + setPrecision(offsetZ, 3) + "," + setPrecision(offsetZRot, 3) + "," + setPrecision(offsetYRot, 3) + "," + setPrecision(offsetXRot, 3) + "\r";
	//std::cout << "string : " << wShift << "\n" ;
	status = writeString(wShift);
	if (status != 0)
	{
		ROS_ERROR("Unable to send wshift command");
		return -1;
	}
	sleepms(100);

	// Send a move command
	status = writeString("moves dummypoint\r");
	if (status != 0)
	{
		ROS_ERROR("Unable to send move command");
		return -1;
	}

	//Assuming speed of 10, need this delay. If you change your speed, you can change this delay but be very careful
	sleepms(10000);
}

void serialConnectA255::sleepms(int milliseconds)
{
	usleep(milliseconds * 1000);
}

int serialConnectA255::writeString(string str)
{
	//if connected
	if (fd_ != -1)
	{
		int countSent = write(fd_, str.c_str(), str.length());

		//Check if data write was successfull
		if (countSent < 0)
		{
			ROS_ERROR("Could not write to the serial port properly.");
			return -1;
		}
	}
	else
	{
		ROS_ERROR("Could not send the command over serial port. Check if connected.");
		return -1;
	}

	return 0;
}

int serialConnectA255::readString(string &str)
{
	int countRcv;

	//if connected
	if (fd_ != -1)
	{
		char buf[BUFFER_SIZE + 1] = "";

		str = "";
		int i = 0;
		while ((countRcv = read(fd_, buf, BUFFER_SIZE)) > 0)
		{
			str.append(buf, countRcv);

			//No further data.
			if (countRcv < BUFFER_SIZE)
				break;
		}

		if (countRcv < 0)
		{
			if (errno == EAGAIN) //If there is no data available, the read() returns -1, with errno set to [EAGAIN].
			{
				ROS_WARN("There is no data available to read. Check if connected.");
				return -1;
			}
			else
			{
				ROS_ERROR("Could not receive the response over serial port. Check if connected.");
				return -1;
			}
		}
	}
	else
	{
		ROS_ERROR("Could not receive the response over serial port. Check if connected.");
		return -1;
	}

	return 0;
}

// Function to go to Ready position which is (0,90,0,0,0) in CRS convention
int serialConnectA255::goReady()
{
	ROS_INFO("Going to 'Ready' position\n");

	int status = writeString("ready\r");
	if (status != 0)
	{
		ROS_ERROR("Unable to go to 'ready' position.\n");
		return -1;
	}
	sleepms(10000);
	return 0;
}

// Function for moving the tool through straight line in the z-direction
int serialConnectA255::goInZ(double desZ)
{
	// Set the joint speed
	ostringstream spConverted;
	spConverted << desZ;  // convert from int to str
	string tempZZ = "";
	tempZZ = "wzs";
	tempZZ = tempZZ + " " + spConverted.str() + "\r";
	cout << tempZZ << "\n";

	int status = writeString(tempZZ);
	if (status != 0)
	{
		ROS_ERROR("Unable to goInZ");
		return -1;
	}
	sleepms(3000);
	tempZZ = "";
	return 0;
}

// Functions for opening and closing the gripper
int serialConnectA255::grip_open()
{
	
	int status = writeString("grip_open\r");
	if(status != 0)
	{
		ROS_ERROR("Unable to send 'grip_open'");
		return -1;
	}
	sleepms(2000);
}

int serialConnectA255::grip_close()
{
	
	int status = writeString("grip_close\r");
	if(status != 0)
	{
		ROS_ERROR("Unable to send 'grip_close'");
		return -1;
	}
	sleepms(2000);
}

void open_and_initialize_arm_controller(serialConnectA255 armController)
{
	if (armController.openPort() < 0)
	{
		ROS_INFO("Something went wrong in opening the serial port. Please see the messages above.");
		return;
	}
	else
	{
		ROS_INFO("Port open successful");
	}


	if (armController.initialize() < 0)
	{
		ROS_ERROR("Initialization Failed");
		return;
	}
}

void close_arm_controller(serialConnectA255 armController)
{
	if (armController.closePort() < 0)
	{
		ROS_INFO("Something went wrong in closing the serial port. Please see the messages above.");
		return;
	}
	else
	{
		ROS_INFO("Port close successful");
	}
}

cv::Mat fetch_image(std::string file_name)
{
	//Right now this function simply takes the stock image from the directory
	//It can be updated time permitting later to take in an image from the workspace
	Mat image;
	image = imread(file_name, 1);

	if(!image.data)
	{
		printf("Image not loaded properly\n");
	}

	resize(image, image, Size(800, 600));

	return image;
}

void display_image_to_screen(cv::Mat image_to_be_displayed, cv::Mat second_image_to_be_displayed=cv::Mat(30, 40, DataType<float>::type))//, bool second_image)
{
	imshow("Title", image_to_be_displayed);
	//if(second_image)
	//{
		imshow("Second Title", second_image_to_be_displayed);
	//}
	waitKey(10000);
}

cv::Mat create_vectorized_image(cv::Mat image_to_be_vectorized, std::string colour)
{

	cv::Mat edges;
	cv::Canny(image_to_be_vectorized, edges, 95, 100);
	display_image_to_screen(edges);
	cv::Mat dx, dy;
	// sobel derivative approximation X direction of edges image
	cv::Sobel(edges, dx, CV_32F, 1, 0);
	// sobel derivative approximation Y direction of edges image
	cv::Sobel(edges, dy, CV_32F, 0, 1);

	vector<Vec4i> lines;
	// Find hough lines 
	HoughLinesP(edges, lines, 1, CV_PI / 180, 20, 20, 10);

	printf("Canny, sobel, and hough complete\n");
	// Prepare blank mat with same sizes as image
	cv::Mat Blank(image_to_be_vectorized.rows, image_to_be_vectorized.cols, CV_8UC3, Scalar(0, 0, 0));
	printf("Blank image to be drawn on created\n");
   	// Draw lines into image and Blank images
	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::Vec4i l = lines[i];

		//printf("Drawing lines\n");
		//This line was in the original code
		//It however mucks up the original image
		//line(image_to_be_vectorized, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 0), 2, CV_AA);
		if(colour == "BLUE")
		{
			line(Blank, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 2, CV_AA);
		}
		else if (colour == "GREEN")
		{
			line(Blank, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 2, CV_AA);
		}
		else if (colour == "RED")
		{
			line(Blank, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, CV_AA);
		}
		else if (colour == "WHITE")
		{
			line(Blank, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 2, CV_AA);
		}
		else
		{
			printf("Colour argument must be string RED BLUE GREEN or WHITE\n");
		}
	}
	return Blank;
}

cv::vector<Vec4i> generate_vector_of_lines(cv::Mat image_to_be_vectorized)
{
	cv::Mat edges;
	cv::Canny(image_to_be_vectorized, edges, 95, 100);
	
	cv::Mat dx, dy;
	// sobel derivative approximation X direction of edges image
	cv::Sobel(edges, dx, CV_32F, 1, 0);
	// sobel derivative approximation Y direction of edges image
	cv::Sobel(edges, dy, CV_32F, 0, 1);

	vector<Vec4i> lines;
	// Find hough lines 
	HoughLinesP(edges, lines, 1, CV_PI / 180, 100, 100, 10);

	return lines;
}

void save_image(cv::Mat image_to_be_saved, std::string file_name)
{
	file_name = file_name+".jpg";
	imwrite(file_name, image_to_be_saved);
}

cv::vector<Vec6f> vectors_2d_to_3d(cv::vector<Vec4i> two_d_vector, double scale, double offsetX, double offsetY, double heightZ)
{
	//Convert the 2d vector of pixel points to a 3d vector XYZ co-ordinates for the robot to move to and from
	//This is where typecast occurs as well to move from pixel integers to double co-ordinates
	//Typecasting is preformed by multiply	ing the ints by the double "scale"

	double startX, startY, startZ, endX, endY, endZ;
	std::vector<Vec6f> three_d_vector;
	for(size_t i = 0; i < two_d_vector.size(); i++)
	{
		cv::Vec4i line = two_d_vector[i];
		startX = (line[0]-640/2)*scale+offsetX;
		startY = (line[1]-480/2)*scale+offsetY;
		startZ = heightZ;
		endX = (line[2]-640/2)*scale+offsetX;
		endY = (line[3]-480/2)*scale+offsetY;
		endZ = heightZ;
		cv::Vec6f newVector(startX, startY, startZ, endX, endY, endZ);
		three_d_vector.push_back(newVector);
	}

	return three_d_vector;
}

std::vector<Mat> seperate_channels(cv::Mat src)
{
	vector<Mat> spl;
	std::vector<Mat> result;
	cv::split(src,spl);

	// Create an zero pixel image for filling purposes - will become clear later
	// Also create container images for B G R channels as colour images
	//Mat empty_image = new Mat::zeros(src.rows, src.cols, CV_8UC1);
	cv::Mat empty_image(src.rows, src.cols, CV_8UC1, Scalar(0));
	Mat result_blue(src.rows, src.cols, CV_8UC3); // notice the 3 channels here!
	Mat result_green(src.rows, src.cols, CV_8UC3); // notice the 3 channels here!
	Mat result_red(src.rows, src.cols, CV_8UC3); // notice the 3 channels here!

	// Create blue channel
	Mat in1[] = { spl[0], empty_image, empty_image };
	int from_to1[] = { 0,0, 1,1, 2,2 };
	mixChannels( in1, 3, &result_blue, 1, from_to1, 3 );

	// Create green channel
	Mat in2[] = { empty_image, spl[1], empty_image };
	int from_to2[] = { 0,0, 1,1, 2,2 };
	mixChannels( in2, 3, &result_green, 1, from_to2, 3 );

	// Create red channel
	Mat in3[] = { empty_image, empty_image, spl[2]};
	int from_to3[] = { 0,0, 1,1, 2,2 };
	mixChannels( in3, 3, &result_red, 1, from_to3, 3 );

	result.push_back(result_blue);
	result.push_back(result_green);
	result.push_back(result_red);
	return result;
}

cv::Mat blur_image(cv::Mat 	image_to_be_blurred, int strength) 
{
	Mat dst;
    GaussianBlur( image_to_be_blurred, dst, Size( strength, strength ), 0, 0);
    return dst;
}

cv::Mat draw_circles(cv::Mat image_to_have_circles_draw, std::vector<Vec3f> circles)
{
	cv::Mat result;
	result = image_to_have_circles_draw;
	for(size_t i = 0; i < circles.size(); i++) 
	{
		Point centre(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);

		//Draw small circle at centre
		circle(result, centre, 3, 255, -1, 8, 0);

		//Draw circle of correct radius
		circle(result, centre, radius, 255, 3, 8 ,0);
	}
	return result;
}

void list_circle_xyr(std::vector<Vec3f> v)
{
	for(size_t i = 0; i < v.size(); i++)
	{
		cout << "X: " << v[i][0] << " Y: " << v[i][1] << " R: " << v[i][2] << endl;
	}
	return;
}

std::vector<Vec3f> sort_circles(std::vector<Vec3f> v)
{
	int size;
	size = v.size();
	Vec3f temp;
    for(int i=0; i<size; i++)
    {
        for(int j=size-1; j>i; j--)
        {
            if(v[j][0]<v[j-1][0])
            {
                temp=v[j-1];
                v[j-1]=v[j];
                v[j]=temp;
            }
        }
    }
    return v;
}

cv::Mat maximize_contrast(cv::Mat image, int threshold) 
{
	cv::Mat new_image = Mat::zeros(image.size(), image.type());
	for(size_t y = 0; y < image.cols; y++)
	{
		for(size_t x = 0; x < image.rows; x++)
		{
			for( size_t c = 0; c < 3; c++) 
			{
				if(image.at<Vec3b>(x,y)[c] < threshold) 
				{
					new_image.at<Vec3b>(x,y)[c] = 0;
				}
				else
				{
					new_image.at<Vec3b>(x,y)[c] = 255;
				}

			}

		}
	}	
	return new_image;
}

cv::Mat contrast_and_brightness(cv::Mat image, double alpha, int beta)
{
	 Mat new_image = Mat::zeros( image.size(), image.type() );

	 /// Do the operation new_image(i,j) = alpha*image(i,j) + beta
	 for( int y = 0; y < image.rows; y++ )
	    { for( int x = 0; x < image.cols; x++ )
	         { for( int c = 0; c < 3; c++ )
	              {
	      new_image.at<Vec3b>(y,x)[c] =
	         saturate_cast<uchar>( alpha*( image.at<Vec3b>(y,x)[c] ) + beta );
	             }
	    }
	    }
	return new_image;
}


std::vector<Vec3f> filter_circles(std::vector<Vec3f> v) 
{
	std::vector<Vec3f> result;

	for(size_t i = 0; i < v.size(); i++)
	{
		float x = v[i][0];
		float y = v[i][1];
		float r = v[i][2];
		//result.push_back(v[i]);
		if(y > 220.0 && y < 260.0 && x > 280.0 && x < 420.0)
		{
			result.push_back(v[i]);
			cout << v[i] << endl;
		}
	}

	return result;
}


cv::Mat image2;
bool got_marker_image = false;
void x_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (got_marker_image) return;

	ROS_INFO("Got Image");
  try
  {
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image2);
    image2 = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  got_marker_image = true;
}

cv::Vec3f camPoint_to_CRSvector(std::vector<Vec3f> markers, int marker_number)
{
	double xpix, ypix;
	xpix = 0.00000081;
	ypix = 0.00000081;

	int i, j;
	i = markers[marker_number][0]-640/2;
	j = markers[marker_number][1]-480/2;

	double w, f, u, v;
	f = 0.0043;
	w = 0.9;
	u = i*xpix*(f - w)/f;
	v = j*ypix*(f - w)/f;

	float x, y, z;
	x = v+0.515;
	y = u+0.890;
	z = 0.09;//0.9943-w-f;//-w+w+f;

	cv::Vec3f marker_vec;
	marker_vec[0] = x;
	marker_vec[1] = y;
	marker_vec[2] = z;

	return marker_vec;
}

void Move_To_Position(Vec3f pos)
{
	ROS_INFO("Moving to Position:");
	ROS_INFO("(x,y,z)[mm] = (%f, %f, %f)", pos[0]*1000, pos[1]*1000, pos[2]*999);
	int temp = serial->moveRobot(pos[0]*1000, pos[1]*1000, pos[2]*1000, 0, 0, 90);
}

void Grab_Marker(cv::Vec3f marker_point)
{
	ROS_INFO("Grabbing Marker");
	cv::Vec3f marker_hover_point = marker_point;
	marker_hover_point[2] = (float)marker_hover_point[2] + 0.05;

	Move_To_Position(marker_hover_point);
	//sleepms(1000);
	Move_To_Position(marker_point);
	//sleepms(1000);
	ROS_INFO("Closing");
	serial->grip_close();
	//sleepms(1000);
	Move_To_Position(marker_hover_point);
	//sleepms(1000);
}

void Place_Marker(cv::Vec3f marker_point)
{
	ROS_INFO("Placing Marker");
	cv::Vec3f marker_hover_point = marker_point;
	marker_hover_point[2] = (float)marker_hover_point[2] + 0.05;

	Move_To_Position(marker_hover_point);
	//sleepms(1000);
	Move_To_Position(marker_point);
	//sleepms(1000);
	serial->grip_open();
	//sleepms(1000);
	Move_To_Position(marker_hover_point);
	//sleepms(1000);
}

void Draw_Line(std::vector<cv::Vec6f> lines)
{
	// sx, sy, ex, ey
	ROS_INFO("Drawing Line");
	double scale = 0.001;
	double draw_Z = 0.4;
	double hover_Z = 0.3;

	double ik_angles[5] = {};

	double draw_yaw = 0.0;

	int speed = 10;


	// Iterate through points to go to
	for(int i = 0; i < lines.size(); i++)
	{
		cv::Vec6f point = lines.at(i);
		
		// Get first line point
		cv::Vec3f start;
		start[0] = point[0];
		start[1] = point[1];
		start[2] = point[2];

		// Get end point
		cv::Vec3f end;
		end[0] = point[3];
		end[1] = point[4];
		end[2] = point[5];

		// Make hover points
		cv::Vec3f start_hover = start;
		start_hover[2] = hover_Z;

		cv::Vec3f end_hover = end;
		start_hover[2] = hover_Z;

		// Append: start_hover, start, end, end_hover
		std::vector<cv::Vec3f> waypoints;
		waypoints.push_back(start_hover);
		waypoints.push_back(start);
		waypoints.push_back(end);
		waypoints.push_back(end_hover);

		for(int j = 0; i< waypoints.size(); j++)
		{
			cv::Vec3f waypoint = waypoints.at(j);
			/*
			ik_angles = serial->invKine(waypoint[0], waypoint[1], waypoint[2], draw_yaw);
			serial->moveTheta(speed, ik_angles[0], ik_angles[1], ik_angles[2], ik_angles[3], ik_angles[4]);
			*/
			Move_To_Position(waypoint);
			//serial->moveRobot(waypoint[0], waypoint[1], waypoint[2], 0, 0, 90);
		}
	}
}

void Draw_Colored_Lines(cv::Mat source, cv::Vec3f marker_point)
{
	ROS_INFO("Executing Draw Line");
	double scale = 1.0;
	double offsetX = 0.27;
	double offsetY = 0.0;
	double heightZ = 0.0;

	cv::vector<cv::Vec4i> pixels = generate_vector_of_lines(source);
	cv::vector<cv::Vec6f> lines = vectors_2d_to_3d(pixels , scale, offsetX, offsetY, heightZ);

	serial->goReady();

	Grab_Marker(marker_point);
	Draw_Line(lines);
	Place_Marker(marker_point);

	serial->goReady();
}







//MAIN LOOP
int main(int argc, char **argv)
{

	printf("Hello World\n");
	ros::init(argc, argv, "serial");

	serial = new serialConnectA255();
	serial->openPort();
	serial->initialize();

	serial->moveRobot(300, 0.0, 300, 0, 0, 90);

	// //System.out.println(Core.getBuildInformation());
	Mat image;
	Mat vector_image;
	Mat blurred_image;
	std::vector<Mat> bgr;
	Mat blue;
	Mat green;
	Mat red;	
	Mat final_result;
	int blur_strength = 9;
	std::string image_to_draw_file_name = "/home/me-547/Desktop/8.png";
	std::string workplace_image_test = "/home/me547/Desktop/8.png";



	image = fetch_image(image_to_draw_file_name);
	display_image_to_screen(image);
	blurred_image = blur_image(image, blur_strength);
	display_image_to_screen(blurred_image);

	bgr = seperate_channels(blurred_image);

	blue = create_vectorized_image(bgr[0], "BLUE");
	green = create_vectorized_image(bgr[1], "GREEN");
	red = create_vectorized_image(bgr[2], "RED");
	final_result = red + blue + green;
	final_result = Scalar(255, 255, 255) - final_result;
	display_image_to_screen(final_result);
	display_image_to_screen(red);

	//Workspace image to find marker in
	//Currently a test image. 
	
	std::vector<Mat> bgr2;
	cv::Mat blue2;
	cv::Mat green2;
	cv::Mat red2;
	cv::Mat image2_unfiltered;

	


	ros::NodeHandle n_ = ros::NodeHandle("Serial");
	//cv::namedWindow("overhead_view");
	//cv::startWindowThread();
	image_transport::ImageTransport it(n_);
	image_transport::Subscriber sub = it.subscribe("/webcam/image_raw", 15, x_imageCallback);



	ROS_INFO("Looping");
	while(!got_marker_image)
	{
		ros::spinOnce();
		ROS_INFO("*");

	}

	cout << "Finding markers in the scene" << endl;
	//image2 = fetch_image(workplace_image_test);
	image2_unfiltered = image2;
	display_image_to_screen(image2_unfiltered);
	//image2 = maximize_contrast(image2, 50);
	// display_image_to_screen(image2);
	//bgr2 = seperate_channels(image2);
	//image2 = contrast_and_brightness(image2, 2.0, 0);
	cout << "Starting grayscale conversion" << endl;
	//Convert all three channels to gray, and blur to avoid false detection
	//cvtColor(bgr2[0], blue2, CV_BGR2GRAY);
	//cvtColor(bgr2[1], green2, CV_BGR2GRAY);
	//cvtColor(bgr2[2], red2, CV_BGR2GRAY);
	// GaussianBlur(blue2, blue2, Size(9,9), 2, 2);
	// GaussianBlur(green2, green2, Size(9,9), 2, 2);	
	// GaussianBlur(red2, red2, Size(9,9), 2, 2);
	// display_image_to_screen(blue2);
	// display_image_to_screen(green2);
	// display_image_to_screen(red2);

	cvtColor(image2, image2, CV_BGR2GRAY);
	//GaussianBlur(image2, image2, Size(5,5), 2, 2);


	//Store the circles in a vector of three floats for each channel
	cout << "Running HoughCircles" << endl;
	std::vector<Vec3f> circles;
	std::vector<Vec3f> circles_filtered;

	// std::vector<Vec3f> blue_circles;
	// std::vector<Vec3f> green_circles;
	// std::vector<Vec3f> red_circles;
	// std::vector<Vec3f> blue_circles_filtered;
	// std::vector<Vec3f> green_circles_filtered;
	// std::vector<Vec3f> red_circles_filtered;

	int minimum_dist_btwn_centres = 15;
	int upper_internal_canny_threshold = 40;
	int threshold_centre_detect = 4;
	int min_radius = 6;
	int max_radius = 14;

	HoughCircles(image2, circles, CV_HOUGH_GRADIENT, 1, minimum_dist_btwn_centres, upper_internal_canny_threshold, threshold_centre_detect, min_radius, max_radius);
	circles_filtered = filter_circles(circles);

	// HoughCircles(blue2, blue_circles, CV_HOUGH_GRADIENT, 1, minimum_dist_btwn_centres, upper_internal_canny_threshold, threshold_centre_detect, min_radius, max_radius);
	// HoughCircles(green2, green_circles, CV_HOUGH_GRADIENT, 1, minimum_dist_btwn_centres, upper_internal_canny_threshold, threshold_centre_detect, min_radius, max_radius);
	// HoughCircles(red2, red_circles, CV_HOUGH_GRADIENT, 1, minimum_dist_btwn_centres, upper_internal_canny_threshold, threshold_centre_detect, min_radius, max_radius);
	// blue_circles_filtered = filter_circles(blue_circles);
	// green_circles_filtered = filter_circles(green_circles);
	// red_circles_filtered = filter_circles(red_circles);

	cout << "Displaying results of HoughCircles" << endl;
	// cout << blue_circles[0][0] << endl;
	// cout << green_circles[0][0] << endl;
	// cout << red_circles[0][0] << endl;
	//Draw markers on the images of the circles
	// cv::Mat blue_with_circles;
	// cv::Mat green_with_circles;
	// cv::Mat red_with_circles;
	cv::Mat image2_with_circles;

	cout << "Drawing circles" << endl;		
	// blue_with_circles = draw_circles(blue2, blue_circles_filtered);
	// green_with_circles = draw_circles(green2, green_circles_filtered);
	// red_with_circles =  draw_circles(red2, red_circles_filtered);
	image2_with_circles = draw_circles(image2, circles_filtered);
	//Display the results
	cout << "Displaying the final images" << endl;
	// display_image_to_screen(blue_with_circles);
	// display_image_to_screen(green_with_circles);
	// display_image_to_screen(red_with_circles);
	display_image_to_screen(image2_with_circles, image2_unfiltered);
	save_image(image2_unfiltered, "Markers");
	cout << "Blue Circles" << endl;
	circles_filtered = sort_circles(circles_filtered);
	list_circle_xyr(circles_filtered);
	
	cout << "go get the markers" << endl;
	cv::Vec3f red_marker, blue_marker, green_marker;

	red_marker = camPoint_to_CRSvector(circles_filtered, 0);
	green_marker = camPoint_to_CRSvector(circles_filtered, 1);
	blue_marker = camPoint_to_CRSvector(circles_filtered, 2);



	Draw_Colored_Lines(red, red_marker);
	
	cout << "complete" << endl;
	serial->closePort();
}
