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
#include "colors.h"
#include "variables.h"

using namespace cv;
using namespace std;
using namespace boost;

#define GRIPPER_LENGTH 195


void Move_To_Position(Vec3f pos)
{
    ROS_INFO("  Moving to Position:");
    float theta = atan2(pos[1], pos[0]);
    float gripper_x = GRIPPER_LENGTH*cos(theta);
    float gripper_y = GRIPPER_LENGTH*sin(theta);
    ROS_INFO("    (x,y,z)[mm] = (%f, %f, %f)", pos[0]*1000, pos[1]*1000, pos[2]*999);
    int temp = serial->moveRobot(pos[0]*1000-gripper_x, pos[1]*1000-gripper_y, pos[2]*1000, 0, 0, 90);

    if(temp == 0)
    {
        serial->sleepms(3000);
        ROS_INFO("Successful position request");
    }
    else if(temp == -1)
    {
        ROS_INFO("Position request outside the workspace");
    }
    else
    {
        ROS_INFO("ERROR in Move_To_Position function");
    }
}

void Move_To_Position_Draw(Vec3f pos)
{
    ROS_INFO("  Moving to Position:");
    float theta = atan2(pos[1], pos[0]);
    float gripper_x = GRIPPER_LENGTH*cos(theta);
    float gripper_y = GRIPPER_LENGTH*sin(theta);
    ROS_INFO("    (x,y,z)[mm] = (%f, %f, %f)", pos[0]*1000, pos[1]*1000, pos[2]*999);
    int temp = serial->moveRobot(pos[0]*1000-gripper_x, pos[1]*1000-gripper_y, pos[2]*1000, 0, 0, 90);

    if(temp == 0)
    {
        serial->sleepms(1000);
        ROS_INFO("Successful position request");
    }
    else if(temp == -1)
    {
        ROS_INFO("Position request outside the workspace");
    }
    else
    {
        ROS_INFO("ERROR in Move_To_Position function");
    }
}


Vec3f Grab_Marker(cv::Vec3f marker_point)
{
    ROS_INFO("Grabbing Marker");
    serial->grip_open ();
    cv::Vec3f marker_hover_point = marker_point;
    marker_hover_point[2] = (float)marker_hover_point[2] + 0.1;

    Move_To_Position(marker_hover_point);

    Vec3f correction = JoystickControl(marker_hover_point);

    Move_To_Position(marker_point + correction);
    serial->grip_close();

    marker_hover_point[2] += 0.05;
    Move_To_Position(marker_hover_point + correction);
    //sleepms(1000);

    return correction;
}

Vec3f JoystickControl(cv::Vec3f base)
{
    cout << "ADJUSTING POSITTION; USE WASD, Q TO EXIT" << endl;
    char input = 'x';

    cv::Vec3f diff(0, 0, 0);

    const float delta = 0.003;

    do
    {
        cout << "Enter new input" << endl;
        cin >> input;

        switch(input)
        {
            case 'w':
                diff[1] += delta;
                break;
            case 's':
                diff[1] -= delta;
                break;
            case 'd':
                diff[0] += delta;
                break;
            case 'a':
                diff[0] -= delta;
                break;

            case 'i':
                diff[1] += 3*delta;
                break;
            case 'k':
                diff[1] -= 3*delta;
                break;
            case 'l':
                diff[0] += 3*delta;
                break;
            case 'j':
                diff[0] -= 3*delta;
                break;

            case 'q':
                break;
            default:
                break;
        }


        if (input != 'q')
        {
            Move_To_Position_Draw(base + diff);
        }

    }while (input != 'q');

    return diff;
}

void Place_Marker(cv::Vec3f marker_point)
{
    ROS_INFO("Placing Marker");
    cv::Vec3f marker_hover_point = marker_point;
    marker_hover_point[2] = (float)marker_hover_point[2] + 0.1;

    Move_To_Position(marker_hover_point);
    Move_To_Position(marker_point);
    serial->grip_open();
    Move_To_Position(marker_hover_point);
    //sleepms(1000);
}

void Draw_Line(std::vector<cv::Vec6f> lines)
{
    // sx, sy, ex, ey
    ROS_INFO("Drawing Lines");
    double scale = 0.001;
    double draw_Z = 0.05;
    double hover_Z = 0.1;

    double ik_angles[5] = {};

    double draw_yaw = 0.0;

    int speed = 10;


    cout << "Number of lines is: " << lines.size() << endl;
    //ROS_INFO("Number of lines to be drawn is: "+lines.size());
    // Iterate through points to go to
    for(int i = 0; i < lines.size(); i++)
    {
        ROS_INFO("Drawing line %d of %d", i, (int)lines.size());

        //ROS_INFO("Drawing line number: %i", i+1);
        cv::Vec6f point = lines.at(i);
        
        // Get first line point
        cv::Vec3f start;
        start[0] = point[0];
        start[1] = point[1];
        start[2] = draw_Z;

        // Get end point
        cv::Vec3f end;
        end[0] = point[3];
        end[1] = point[4];
        end[2] = draw_Z;

        // Make hover points
        cv::Vec3f start_hover = start;
        start_hover[2] = hover_Z;

        cv::Vec3f end_hover = end;
        end_hover[2] = hover_Z;
        if (ros::ok())
        {
            cout << "Segment " << i << " of " << lines.size() << endl;
            ROS_INFO("      Drawing Start start_hover");
            Move_To_Position_Draw(start_hover);
            ROS_INFO("      Drawing Start");
            Move_To_Position_Draw(start);
            ROS_INFO("      End Point");
            Move_To_Position_Draw(end);
            ROS_INFO("      End Hover Point");
            Move_To_Position_Draw(end_hover);
        }
        /*
        for(int j = 0; i< waypoints.size(); j++)
        {
            cv::Vec3f waypoint = waypoints.at(j);
            /*
            ik_angles = serial->invKine(waypoint[0], waypoint[1], waypoint[2], draw_yaw);
            serial->moveTheta(speed, ik_angles[0], ik_angles[1], ik_angles[2], ik_angles[3], ik_angles[4]);
            *
            Move_To_Position(waypoint);
            //serial->moveRobot(waypoint[0], waypoint[1], waypoint[2], 0, 0, 90);
        }
        */
    }
}

void Draw_Colored_Lines(cv::Mat source, cv::Vec3f marker_point)
{
    ROS_INFO("Executing Draw Line");
    double scale = 0.00032;
    double offsetX = 0.55;
    double offsetY = 0.00;
    double heightZ = 0.0;

    display_image_to_screen(source);
    cv::vector<cv::Vec4i> pixels = generate_vector_of_lines(source);
    cv::vector<cv::Vec6f> lines = vectors_2d_to_3d(pixels , scale, offsetX, offsetY, heightZ);
    cout << "Displaying list of lines to be drawn" << endl;
    for(int i = 0; i < lines.size(); i++)
    {
        cout << lines[i][0] << " " << lines[i][1] << " " << lines[i][2] << " " << lines[i][3] << " " << lines[i][4] << " "  << lines[i][5] << endl;
    }
    //cout << lines << endl;
    serial->goReady();

    serial->setSpeed(20);

    Vec3f correction = Grab_Marker(marker_point);
    Draw_Line(lines);
    Place_Marker(marker_point + correction);

    serial->setSpeed(10);

    serial->goReady();
}


