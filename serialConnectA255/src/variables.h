#ifndef VARIABLES_H_
#define VARIABLES_H_

#include "serial.h"

extern cv::Mat image2;
extern bool got_marker_image;
extern serialConnectA255* serial;
extern int clickX;
extern int clickY;
extern bool flagImageClickReceived;

#endif