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



void Draw_Colored_Lines(cv::Mat source, int marker_index)
{
	double scale = 1.0;
	double offsetX = 0.27;
	double offsetY = 0.0;
	double heightZ = 0.0;

	cv::vector<Vec4i> pixels = generate_vector_of_lines(cv::Mat source);
	cv::vector<Vec6f> lines = vectors_2d_to_3d(pixels , scale, offsetX, offsetY, heightZ);

	serial.goReady();

	Grab_Marker(marker_index);
	Draw_Line(lines);
	Place_Marker(marker_index);

	serial.goReady();
}

void Grab_Marker(int index)
{
	cv::vector<Vec3f> marker_point;
	// Get marker_point
	cv::vector<Vec3f> marker_hover_point = marker_point;
	marker_hover_point[2] = marker_hover_point[2] + 0.05;

	Move_To_Position(marker_hover_point);
	sleepms(1000);
	Move_To_Position(marker_point);
	sleepms(1000);
	serial.grip_close();
	sleepms(1000);
	Move_To_Position(marker_hover_point);
	sleepms(1000);
}

void Place_Marker(int index)
{
	cv::vector<Vec3f> marker_point;
	// Get marker_point
	cv::vector<Vec3f> marker_hover_point = marker_point;
	marker_hover_point[2] = marker_hover_point[2] + 0.05;

	Move_To_Position(marker_hover_point);
	sleepms(1000);
	Move_To_Position(marker_point);
	sleepms(1000);
	serial.grip_open();
	sleepms(1000);
	Move_To_Position(marker_hover_point);
	sleepms(1000);
}

void Move_To_Position(Vec3f pos)
{
	serial.moveRobot(pos[0], pos[1], pos[2], 0, 0, 90)
}

void Draw_Line(std::vector<Vec6f> lines)
{
	// sx, sy, ex, ey

	double scale = 1;
	double draw_Z = 0;
	double hover_Z = 0;

	double ik_angles[5] = {};

	double draw_yaw = 0.0;

	int speed = 10;


	// Iterate through points to go to
	for(int i = 0; i < lines.size(); i++0)
	{
		std::Vec6f point = lines.at(i);
		
		// Get first line point
		std::Vec3f start;
		start[0] = point[0];
		start[1] = point[1];
		start[2] = point[2];

		// Get end point
		std::Vec3f end;
		end[0] = point[3];
		end[1] = point[4];
		end[2] = point[5];

		// Make hover points
		std::Vec3f start_hover = start;
		start_hover[2] = hover_Z;

		std::Vec3f end_hover = end;
		start_hover[2] = end_Z;

		// Append: start_hover, start, end, end_hover
		std::vector<Vec3f> waypoints;
		waypoints.push_back(start_hover);
		waypoints.push_back(start);
		waypoints.push_back(end);
		waypoints.push_back(end_hover);

		for(int j = 0; i< waypoints.size(); j++)
		{
			waypoint = waypoints.at(j);
			/*
			ik_angles = serial.invKine(waypoint[0], waypoint[1], waypoint[2], draw_yaw);
			serial.moveTheta(speed, ik_angles[0], ik_angles[1], ik_angles[2], ik_angles[3], ik_angles[4]);
			*/
			serial.moveRobot(waypoint[0], waypoint[1], waypoint[2], 0, 0, 90);
		}
	}

