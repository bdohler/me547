#ifndef ROBOTIX_H_
#define ROBOTIX_H_

#include "opencv2/opencv.hpp"

#include "variables.h"
#include "serial.h"

void Draw_Colored_Lines(cv::Mat source, cv::Vec3f marker_point);

void Move_To_Position(Vec3f pos);
void Grab_Marker(cv::Vec3f marker_point);
void Place_Marker(cv::Vec3f marker_point);
void Draw_Line(std::vector<cv::Vec6f> lines);
#endif