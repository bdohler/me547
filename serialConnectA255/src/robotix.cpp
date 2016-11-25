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

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "robotix.h"


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


