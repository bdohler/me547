#ifndef COLORS_H_
#define COLORS_H_

#include "opencv2/opencv.hpp"

cv::Mat fetch_image(std::string file_name);

void display_image_to_screen(cv::Mat image_to_be_displayed, cv::Mat second_image_to_be_displayed=cv::Mat(30, 40, DataType<float>::type))//, bool second_image);

cv::Mat create_vectorized_image(cv::Mat image_to_be_vectorized, std::string colour);

cv::vector<Vec4i> generate_vector_of_lines(cv::Mat image_to_be_vectorized);

void save_image(cv::Mat image_to_be_saved, std::string file_name);

cv::vector<Vec6f> vectors_2d_to_3d(cv::vector<Vec4i> two_d_vector, double scale, double offsetX, double offsetY, double heightZ);

std::vector<Mat> seperate_channels(cv::Mat src);

cv::Mat blur_image(cv::Mat  image_to_be_blurred, int strength) ;

cv::Mat draw_circles(cv::Mat image_to_have_circles_draw, std::vector<Vec3f> circles);

void list_circle_xyr(std::vector<Vec3f> v);

std::vector<Vec3f> sort_circles(std::vector<Vec3f> v);

cv::Mat maximize_contrast(cv::Mat image, int threshold);
cv::Mat contrast_and_brightness(cv::Mat image, double alpha, int beta);


std::vector<Vec3f> filter_circles(std::vector<Vec3f> v);

cv::Vec3f camPoint_to_CRSvector(std::vector<Vec3f> markers, int marker_number);


void x_imageCallback(const sensor_msgs::ImageConstPtr& msg);


#endif