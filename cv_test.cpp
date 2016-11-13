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

cv::Mat fetch_image()
{
	//Right now this function simply takes the stock image from the directory
	//It can be updated time permitting later to take in an image from the workspace
	cv::Mat image;
	image = imread("8.png", 1);

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
	printf("Creating seperate blue channel\n");
	Mat in1[] = { spl[0], empty_image, empty_image };
	int from_to1[] = { 0,0, 1,1, 2,2 };
	mixChannels( in1, 3, &result_blue, 1, from_to1, 3 );

	// Create green channel
	printf("Creating seperate green channel\n");
	Mat in2[] = { empty_image, spl[1], empty_image };
	int from_to2[] = { 0,0, 1,1, 2,2 };
	mixChannels( in2, 3, &result_green, 1, from_to2, 3 );

	// Create red channel
	printf("creating seperate red channel\n");
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

int main(int argc, const char** argv)
{
	printf("Hello World\n");

	//System.out.println(Core.getBuildInformation());
	Mat image;
	Mat vector_image;
	Mat blurred_image;
	std::vector<Mat> bgr;
	Mat blue;
	Mat green;
	Mat red;	
	Mat final_result;
	image = fetch_image();
	int blur_strength = 9;

	display_image_to_screen(image);
	blurred_image = blur_image(image, blur_strength);
	display_image_to_screen(blurred_image);

	printf("Starting channel seperation\n");
	bgr = seperate_channels(blurred_image);
	printf("Splitting of image complete\n");

	blue = create_vectorized_image(bgr[0], "BLUE");
	printf("Blue image vectorized\n");
	green = create_vectorized_image(bgr[1], "GREEN");
	printf("Green image vectorized\n");
	red = create_vectorized_image(bgr[2], "RED");
	printf("Red image vectorized\n");
	printf("Creation of vectorized BGR complete\n");
	final_result = red + blue + green;
	final_result = Scalar(255, 255, 255) - final_result;
	display_image_to_screen(final_result);
	display_image_to_screen(red);

	// cv::Mat edges;

 // 	// Canny edge 
	// cv::Canny(image, edges, 95, 100);

	// cv::Mat dx, dy;

	// // sobel derivative approximation X direction of edges image
	// cv::Sobel(edges, dx, CV_32F, 1, 0);

	// // sobel derivative approximation Y direction of edges image
	// cv::Sobel(edges, dy, CV_32F, 0, 1);

	// vector<Vec4i> lines;
	// // Find hough lines 
	// HoughLinesP(edges, lines, 1, CV_PI / 180, 100, 100, 10);

	// // Prepare blank mat with same sizes as image
	// Mat Blank(image.rows, image.cols, CV_8UC3, Scalar(0, 0, 0));

 //   	// Draw lines into image and Blank images
	// for (size_t i = 0; i < lines.size(); i++)
	// {
	// 	Vec4i l = lines[i];

	// 	line(image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 0), 2, CV_AA);

	// 	line(Blank, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 2, CV_AA);

	// }

	// imwrite("houg.jpg", image);
	// imshow("Edges", image);

	// waitKey(10000);

	// imwrite("houg2.jpg", Blank);
	// imshow("Edges Structure", Blank);

}