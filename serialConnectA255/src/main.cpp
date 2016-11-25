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
#include "colors.h"
#include "serial.h"
#include "variables.h"

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

//added namespace for computer vision
using namespace cv;
using namespace std;
using namespace boost;


cv::Mat image2;
bool got_marker_image = false;
serialConnectA255* serial;


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
