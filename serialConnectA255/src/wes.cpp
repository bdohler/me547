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


void Draw_Line(std::vector<Vec6f> lines)
{
	// sx, sy, ex, ey

	double scale = 1;
	double offsetX = 0;
	double offsetY = 0;
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
			ik_angles = serial.invKine(waypoint[0], waypoint[1], waypoint[2], draw_yaw);
			serial.moveTheta(speed, ik_angles[0], ik_angles[1], ik_angles[2], ik_angles[3], ik_angles[4]);
		}
	}

