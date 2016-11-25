#ifndef SERIAL_H_
#define SERIAL_H_

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

using namespace cv;
using namespace std;
using namespace boost;

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

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

void open_and_initialize_arm_controller(serialConnectA255 armController);

void close_arm_controller(serialConnectA255 armController);

#endif