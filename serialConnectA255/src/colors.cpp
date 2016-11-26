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

#include "opencv2/opencv.hpp"


#include "colors.h"
#include "variables.h"

using namespace cv;
using namespace std;
using namespace boost;

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

void display_image_to_screen(cv::Mat image_to_be_displayed, cv::Mat second_image_to_be_displayed)//=cv::Mat(30, 40, DataType<float>::type))//, bool second_image)
{
    imshow("Workspace", image_to_be_displayed);
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
    //display_image_to_screen(edges);
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
    HoughLinesP(edges, lines, 1, CV_PI / 180, 20, 20, 10);

    Vec4i temp;
    temp[0] = 0;
    temp[1] = 0;
    temp[2] = 0;
    temp[3] = image_to_be_vectorized.rows;
    lines.insert(lines.begin(), temp);
    temp[0] = 0;
    temp[1] = image_to_be_vectorized.rows;
    temp[2] = image_to_be_vectorized.cols;
    temp[3] = image_to_be_vectorized.rows;
    lines.insert(lines.begin(), temp);
    temp[0] = image_to_be_vectorized.cols;
    temp[1] = image_to_be_vectorized.rows;
    temp[2] = image_to_be_vectorized.cols;
    temp[3] = 0;
    lines.insert(lines.begin(), temp);
    temp[0] = image_to_be_vectorized.cols;
    temp[1] = 0;
    temp[2] = 0;
    temp[3] = 0;
    lines.insert(lines.begin(), temp);

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
    //Typecasting is preformed by multiply  ing the ints by the double "scale"

    double startX, startY, startZ, endX, endY, endZ;
    std::vector<Vec6f> three_d_vector;
    for(size_t i = 0; i < two_d_vector.size(); i++)
    {
        cv::Vec4i line = two_d_vector[i];
        // startX = (line[0]-800/2)*scale+offsetX;
        // startY = (line[1]-600/2)*scale+offsetY;
        // startZ = heightZ;
        // endX = (line[2]-800/2)*scale+offsetX;
        // endY = (line[3]-600/2)*scale+offsetY;
        startX = (line[1]-600/2)*scale+offsetX;
        startY = (line[0]-800/2)*scale+offsetY;
        startZ = heightZ;
        endX = (line[3]-600/2)*scale+offsetX;
        endY = (line[2]-800/2)*scale+offsetY;
        endZ = heightZ;
        cv::Vec6f newVector(startX, startY, startZ, endX, endY, endZ);
        three_d_vector.push_back(newVector);
    }
    cout << "vectors_2d_to_3d [1]:" << three_d_vector[0][0] << "," << three_d_vector[0][1] << "," << three_d_vector[0][3] << "," << three_d_vector[0][4] << endl;
    cout << "vectors_2d_to_3d [2]:" << three_d_vector[1][0] << "," << three_d_vector[1][1] << "," << three_d_vector[1][3] << "," << three_d_vector[1][4] << endl;
    cout << "vectors_2d_to_3d [3]:" << three_d_vector[2][0] << "," << three_d_vector[2][1] << "," << three_d_vector[2][3] << "," << three_d_vector[2][4] << endl;
    cout << "vectors_2d_to_3d [4]:" << three_d_vector[3][0] << "," << three_d_vector[3][1] << "," << three_d_vector[3][3] << "," << three_d_vector[3][4] << endl;
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

cv::Mat blur_image(cv::Mat  image_to_be_blurred, int strength) 
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

cv::Vec3f camPoint_to_CRSvector(std::vector<Vec3f> markers, int marker_number)
{
    double xpix, ypix;
    xpix = 0.0000081*8/7; //8/7 is a scale factor that was determined through experimental optimization
    ypix = 0.0000081*9/8; //Similarly, 9/8 is a scale factor

    int i, j;
    i = markers[marker_number][0]-640/2;
    j = markers[marker_number][1]-480/2;    

    double w, f, u, v;
    f = 0.0043;
    w = 0.845;
    u = i*xpix*(f - w)/f;
    v = j*ypix*(f - w)/f;

    float x, y, z;
    x = -v+0.525;
    y = -u+0.090;
    z = 0.13;//0.994 3-w-f;//-w+w+f;

    ROS_INFO("X and Y displacement from camera click (x,y)[m]: %f, %f", x, y);

    cv::Vec3f marker_vec;
    marker_vec[0] = x + 0.0575;
    marker_vec[1] = y + 0.040;
    marker_vec[2] = z;

    return marker_vec;
}

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

void onMouse(int evt, int x, int y, int flags, void* param) {
    if(evt == CV_EVENT_LBUTTONDOWN && flagImageClickReceived == false) {
        clickX = x;
        clickY = y;
        ROS_INFO("Got Click");
        flagImageClickReceived = true;
    }
}