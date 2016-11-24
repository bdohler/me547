//#include <Windows.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "stdio.h"
#include <cstdlib>
// #include "opencv2/imgcodecs/imgcodecs.hpp"
// #include "opencv2/videoio/videoio.hpp"

using namespace cv;
using namespace std;

cv::Mat fetch_image(std::string file_name)
{
	//Right now this function simply takes the stock image from the directory
	//It can be updated time permitting later to take in an image from the workspace
	cv::Mat image;
	image = imread(file_name, 1);

	if(!image.data)
	{
		printf("Image not loaded properly\n");
	}

	resize(image, image, Size(800, 600));

	return image;
}

void display_image_to_screen(cv::Mat image_to_be_displayed)
{
	imshow("Title", image_to_be_displayed);

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
		startX = line[0]*scale+offsetX;
		startY = line[1]*scale+offsetY;
		startZ = heightZ;
		endX = line[2]*scale+offsetX;
		endY = line[3]*scale+offsetY;
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


std::vector<Vec3f> filter_circles(std::vector<Vec3f> v) 
{
	std::vector<Vec3f> result;

	for(size_t i = 0; i < v.size(); i++)
	{
		float x = v[i][0];
		float y = v[i][1];
		float r = v[i][2];
		//result.push_back(v[i]);
		if(y < 200.0 && x < 400.0)
		{
			result.push_back(v[i]);
			cout << v[i] << endl;
		}
	}

	return result;
}

int main(int argc, const char** argv)
{
	printf("Hello World\n");

	// //System.out.println(Core.getBuildInformation());
	// Mat image;
	// Mat vector_image;
	// Mat blurred_image;
	// std::vector<Mat> bgr;
	// Mat blue;
	// Mat green;
	// Mat red;	
	// Mat final_result;
	// int blur_strength = 9;
	// std::string image_to_draw_file_name = "6.jpg";
	std::string workplace_image_test = "colour_test.png";



	// image = fetch_image(image_to_draw_file_name);
	// display_image_to_screen(image);
	// blurred_image = blur_image(image, blur_strength);
	// display_image_to_screen(blurred_image);

	// bgr = seperate_channels(blurred_image);

	// blue = create_vectorized_image(bgr[0], "BLUE");
	// green = create_vectorized_image(bgr[1], "GREEN");
	// red = create_vectorized_image(bgr[2], "RED");
	// final_result = red + blue + green;
	// final_result = Scalar(255, 255, 255) - final_result;
	// display_image_to_screen(final_result);
	// display_image_to_screen(red);

	// //Workspace image to find marker in
	// //Currently a test image. 

	cout << "Starting second component" << endl;
	cv::Mat image2;
	std::vector<Mat> bgr2;
	cv::Mat blue2;
	cv::Mat green2;
	cv::Mat red2;
	image2 = fetch_image(workplace_image_test);
	display_image_to_screen(image2);
	image2 = maximize_contrast(image2, 50);
	display_image_to_screen(image2);
	bgr2 = seperate_channels(image2);

	cout << "Starting grayscale conversion" << endl;
	//Convert all three channels to gray, and blur to avoid false detection
	cvtColor(bgr2[0], blue2, CV_BGR2GRAY);
	cvtColor(bgr2[1], green2, CV_BGR2GRAY);
	cvtColor(bgr2[2], red2, CV_BGR2GRAY);
	GaussianBlur(blue2, blue2, Size(31,31), 2, 2);
	GaussianBlur(green2, green2, Size(31,31), 2, 2);	
	GaussianBlur(red2, red2, Size(31,31), 2, 2);
	display_image_to_screen(blue2);
	display_image_to_screen(green2);
	display_image_to_screen(red2);

	//Store the circles in a vector of three floats for each channel
	cout << "Running HoughCircles" << endl;
	std::vector<Vec3f> blue_circles;
	std::vector<Vec3f> green_circles;
	std::vector<Vec3f> red_circles;
	std::vector<Vec3f> blue_circles_filtered;
	std::vector<Vec3f> green_circles_filtered;
	std::vector<Vec3f> red_circles_filtered;

	int minimum_dist_btwn_centres = 30;
	int upper_internal_canny_threshold = 40;
	int threshold_centre_detect = 1;
	int max_radius = 20;
	HoughCircles(blue2, blue_circles, CV_HOUGH_GRADIENT, 1, minimum_dist_btwn_centres, upper_internal_canny_threshold, threshold_centre_detect, 0, max_radius);
	HoughCircles(green2, green_circles, CV_HOUGH_GRADIENT, 1, minimum_dist_btwn_centres, upper_internal_canny_threshold, threshold_centre_detect, 0, max_radius);
	HoughCircles(red2, red_circles, CV_HOUGH_GRADIENT, 1, minimum_dist_btwn_centres, upper_internal_canny_threshold, threshold_centre_detect, 0, max_radius);
	blue_circles_filtered = filter_circles(blue_circles);
	green_circles_filtered = filter_circles(green_circles);
	red_circles_filtered = filter_circles(red_circles);
	cout << "Displaying results of HoughCircles" << endl;
	// cout << blue_circles[0][0] << endl;
	// cout << green_circles[0][0] << endl;
	// cout << red_circles[0][0] << endl;
	//Draw markers on the images of the circles
	cv::Mat blue_with_circles;
	cv::Mat green_with_circles;
	cv::Mat red_with_circles;

	cout << "Drawing circles" << endl;		
	blue_with_circles = draw_circles(blue2, blue_circles_filtered);
	green_with_circles = draw_circles(green2, green_circles_filtered);
	red_with_circles =  draw_circles(red2, red_circles_filtered);

	//Display the results
	cout << "Displaying the final images" << endl;
	display_image_to_screen(blue_with_circles);
	display_image_to_screen(green_with_circles);
	display_image_to_screen(red_with_circles);

	cout << "Blue Circles" << endl;
	list_circle_xyr(blue_circles);

	cout << "complete" << endl;
}


// $875.00
// 1-877-564-5227

