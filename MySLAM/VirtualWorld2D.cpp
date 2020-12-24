#include "VirtualWorld2D.h"


void VirtualWorld2D::Initialize(std::string path_map_img, std::string path_waypoint_txt)
{
	LoadMapfromImage(path_map_img);
	LoadWaypointfromTxt(path_waypoint_txt);

	rover_inworld.Initialize(waypoints[0]);
	rover_inner.Initialize(waypoints[0]);
}



void VirtualWorld2D::run()
{
	// Moving along the waypoint
	int cnt_waypoint = 1;
	while (cnt_waypoint < waypoints.size())
	{
		// get sensor data from real location
		GetSensorData(&rover_inworld);
		// display
		DisplayMap();

		// move next waypoint
		rover_inworld.MoveNextWaypoint(waypoints[cnt_waypoint]);

		float dist_rover_waypointl = CalDistancePose(rover_inworld.pose_last, waypoints[cnt_waypoint]);
		if (dist_rover_waypointl < 3.0)
			cnt_waypoint++;

		// Calculate rover_inner information

		
		

	}

}

void VirtualWorld2D::LoadMapfromImage(std::string path_map_img)
{
	map_img = cv::imread(path_map_img);

	if (!map_img.empty())
	{
		cv::Mat tmp = map_img.clone();
		cv::cvtColor(map_img, tmp, cv::COLOR_BGR2GRAY);

		for (int i = 0; i < tmp.cols; i++)
			for (int j = 0; j < tmp.rows; j++)
			{
				if (tmp.data[j * tmp.cols + i] > 128)
				{
					Pose2D tmp_landmark;
					tmp_landmark.location = cv::Point(i, j);
					tmp_landmark.theta = AdjustAngle(atan2(j, i) * 180.0 / PI);
					landmarks.push_back(tmp_landmark);
				}
			}
	}
}

void VirtualWorld2D::LoadWaypointfromTxt(std::string path_waypoint_txt)
{
	std::ifstream waypoint_txt(path_waypoint_txt);
	while (!waypoint_txt.eof())
	{
		cv::Point pt;
		float pt_theta;
		waypoint_txt >> pt.x >> pt.y >> pt_theta;

		Pose2D tmp;
		tmp.location = pt;
		tmp.theta = AdjustAngle(pt_theta);
		waypoints.push_back(tmp);
	}
}

void VirtualWorld2D::DisplayMap()
{
	// 1. Draw world map
	//	> trajectory of rover
	//	> present location of rover
	//	> sensor measurement area
	//
	// 2. Draw rover view
	//	> view of rover
	//	> measured landmark
	//	> near landmark
	//
	//	3. Concate world map & rover view
	//
	// Todo
	//	> display measured sensor data
	//	> display rover pose what rover self


	// Declaration cv::Mat for display world
	cv::Mat map_display_in_world = map_img.clone();
	cv::Mat map_display_rotate;	// padded and rotate map
	cv::Mat map_display_roverview;

	// 1. Draw world map

	// Draw Observed Landmarks in world map
	Display_DrawLandmark(map_display_in_world, rover_inworld.pose_last, rover_inworld.pose_last.theta, rover_inworld.landmarks_observed);
	
	/*
	for (const Pose2D &landmark : rover_inworld.landmarks_observed)
	{
		float dist_landmark = sqrt(pow(landmark.location.x,2) + pow(landmark.location.y,2));
		float angle_landmark_rad = Deg2Rad(rover_inworld.pose_last.theta + landmark.theta);

		float pt_relative_x = dist_landmark * cos(angle_landmark_rad);
		float pt_relative_y = dist_landmark * sin(angle_landmark_rad);

		int pt_x = rover_inworld.pose_last.location.x + round(pt_relative_x);
		int pt_y = rover_inworld.pose_last.location.y + round(pt_relative_y);

		map_display_in_world.data[3 * pt_y * map_display_in_world.cols + 3 * pt_x] = 0;
		map_display_in_world.data[3 * pt_y * map_display_in_world.cols + 3 * pt_x + 1] = 0;
		map_display_in_world.data[3 * pt_y * map_display_in_world.cols + 3 * pt_x + 2] = 255;
	}
	*/

	// Draw Waypoint in world map
	cv::Point2d pt_pre_waypoint(-1, -1);
	for (const Pose2D& waypoint : waypoints)
	{
		cv::Point pt_waypoint = waypoint.location;
		cv::circle(map_display_in_world, pt_waypoint, 2, cv::Scalar(0, 255, 0), -1);

		if (pt_pre_waypoint != cv::Point2d(-1, -1))
			cv::line(map_display_in_world, pt_waypoint, pt_pre_waypoint, cv::Scalar(255, 0, 0));

		pt_pre_waypoint = pt_waypoint;
	}

	// Draw Rover in world map
	Display_DrawRover(map_display_in_world, rover_inworld.pose_last, cv::Scalar(0, 165, 255));	//

	// Draw Rover Trajectory in world map
	cv::Point2d pt_pre_rover(-1, -1);
	for(int i=0; i< rover_inworld.pose_trajectory.size()-1; i++)
	{
		cv::Point2d pt_rover = rover_inworld.pose_trajectory[i].location;
		cv::circle(map_display_in_world, pt_rover, 3, cv::Scalar(0, 165, 255));

		if (i > 0)
			cv::line(map_display_in_world, pt_rover, pt_pre_rover, cv::Scalar(0, 165, 255));

		pt_pre_rover = pt_rover;
	}


	// 2. Draw rover view
	//	rover is see map in real location. so, view is displayed about rover_inner but sensor data is inworld
	Display_RotateMap(map_img, &map_display_rotate, rover_inworld.pose_last);		// Rect rover view

	// Calculate rover's pose
	int padding = round(map_display_rotate.cols / 2.0);
	cv::Point pt_rover_rotated = CalRotatedPt(rover_inworld.pose_last, round(padding / 2.0)) + cv::Point2d(padding, padding);
	Pose2D pose_rover_rotated = CreatePose(pt_rover_rotated, -90.0);

	// Draw rover in rotated map
	Display_DrawRover(map_display_rotate, pose_rover_rotated, cv::Scalar(0, 165, 255));

	// Draw Landmark in rotated map
	for (auto &landmark : rover_inworld.landmarks_observed)
	{
		float dist_landmark = sqrt(pow(landmark.location.x, 2) + pow(landmark.location.y, 2));
		float angle_landmark_rad = Deg2Rad(landmark.theta - 90.0);

		//rover_inworld.pose_last.theta + 

		float pt_relative_x = dist_landmark * cos(angle_landmark_rad);
		float pt_relative_y = dist_landmark * sin(angle_landmark_rad);

		int pt_x = pt_rover_rotated.x + round(pt_relative_x);
		int pt_y = pt_rover_rotated.y + round(pt_relative_y);

		map_display_rotate.data[3 * pt_y * map_display_rotate.cols + 3 * pt_x] = 0;
		map_display_rotate.data[3 * pt_y * map_display_rotate.cols + 3 * pt_x + 1] = 0;
		map_display_rotate.data[3 * pt_y * map_display_rotate.cols + 3 * pt_x + 2] = 255;
	}
	  

	Display_CropRoverView(map_display_rotate, &map_display_roverview, rover_inworld.pose_last);	// resize rover view

	


	// 3. Concate world and rover view
	cv::Mat resize_rover_view;
	cv::resize(map_display_roverview, resize_rover_view, cv::Size(map_display_in_world.cols, map_display_in_world.rows));
	cv::Mat Display;
	cv::hconcat(map_display_in_world, resize_rover_view, Display);

	cv::imshow("World2D", Display);
	cv::waitKey(100);

}

void VirtualWorld2D::Display_DrawRover(cv::Mat map_display, Pose2D pose, cv::Scalar color)
{
	float max_sensor_range = info_rover.max_sensor_range;
	float max_sensor_fov = info_rover.max_sensor_fov;
	float max_sensor_fov_rad = Deg2Rad(max_sensor_fov);

	cv::Point2d pt_rover = pose.location;
	float theta_rover = pose.theta;
	float theta_rover_rad = Deg2Rad(pose.theta);

	cv::circle(map_display, pt_rover, 6, color, 2);
	cv::Point2d pt_rover_heading = cv::Point2d(round(pt_rover.x + 10.0 * cos(theta_rover_rad)), 
												round(pt_rover.y + 10.0 * sin(theta_rover_rad)));
	cv::line(map_display, pt_rover, pt_rover_heading, color, 2);

	// Draw Sensor Range of Rover
	cv::ellipse(map_display, pt_rover, cv::Size(max_sensor_range, max_sensor_range),
		0, theta_rover - max_sensor_fov, theta_rover + max_sensor_fov, cv::Scalar(0, 165, 255), 1);
	
	cv::Point2d left_end = cv::Point2d(pt_rover.x + round(max_sensor_range * cos(theta_rover_rad - max_sensor_fov_rad)),
										pt_rover.y + round(max_sensor_range * sin(theta_rover_rad - max_sensor_fov_rad)));
	
	cv::Point2d right_end = cv::Point2d(pt_rover.x + round(max_sensor_range * cos(theta_rover_rad + max_sensor_fov_rad)),
										pt_rover.y + round(max_sensor_range * sin(theta_rover_rad + max_sensor_fov_rad)));
	
	cv::line(map_display, pt_rover, left_end, color);
	cv::line(map_display, pt_rover, right_end, color);

}

void VirtualWorld2D::Display_RotateMap(cv::Mat src, cv::Mat *dst, Pose2D pose)
{
	cv::Mat map_padding;
	int padding = src.cols / 2;
	cv::copyMakeBorder(src, map_padding, padding, padding, padding, padding, cv::BORDER_CONSTANT, cv::Scalar(128, 128, 128));

	cv::Point2f center(src.cols / 2.0 + padding, src.rows / 2.0 + padding), pts[4];
	cv::Size2f size(2 * info_rover.max_sensor_range, 2 * info_rover.max_sensor_range);
	float theta = pose.theta + 90.0;
	cv::Mat rotated_matrix = cv::getRotationMatrix2D(center, theta, 1.0);

	cv::warpAffine(map_padding, *dst, rotated_matrix, cv::Size(map_padding.cols, map_padding.rows));
}

void VirtualWorld2D::Display_CropRoverView(cv::Mat src, cv::Mat *dst, Pose2D pose)
{
	int padding = src.cols / 4.0;
	Eigen::MatrixXf matrix_pt_rover_to_center = Eigen::MatrixXf(2, 1);
	matrix_pt_rover_to_center(0) = pose.location.x + padding - src.cols / 2.0;	// shfit rover location by padding, shift origin to image center
	matrix_pt_rover_to_center(1) = pose.location.y + padding - src.rows / 2.0;
	Eigen::MatrixXf matrix_rotate_90deg = create_rotation_matrix(90.0);
	Eigen::MatrixXf matrix_rotate_rover_theta = create_rotation_matrix(pose.theta);
	Eigen::MatrixXf matrix_pt_rover_rotate = matrix_pt_rover_to_center.transpose() * matrix_rotate_rover_theta * matrix_rotate_90deg;
	
	cv::Point2d pt_rover_rotate(round(matrix_pt_rover_rotate(0)), round(matrix_pt_rover_rotate(1)));
	cv::Point2d shift_origine(src.cols / 2.0, src.cols / 2.0);

	cv::Point2d left_top(pt_rover_rotate + shift_origine
		- cv::Point2d(info_rover.max_sensor_range, round(1.5 * info_rover.max_sensor_range)));

	cv::Point2d right_down(pt_rover_rotate + shift_origine
		+ cv::Point2d(info_rover.max_sensor_range, round(0.5 * info_rover.max_sensor_range)));

	cv::Rect rect_rover_view(left_top, right_down);
	cv::rectangle(src, rect_rover_view, cv::Scalar(255, 255, 255));
	*dst = src(rect_rover_view);
}

void VirtualWorld2D::Display_DrawLandmark(cv::Mat dst, Pose2D pose, float theta_basis, std::vector<Pose2D> landmarks)
{
	for (auto& landmark : landmarks)
	{
		float dist_landmark = sqrt(pow(landmark.location.x, 2) + pow(landmark.location.y, 2));
		float angle_landmark_rad = Deg2Rad(landmark.theta + theta_basis);

		//rover_inworld.pose_last.theta + 

		float pt_relative_x = dist_landmark * cos(angle_landmark_rad);
		float pt_relative_y = dist_landmark * sin(angle_landmark_rad);

		int pt_x = pose.location.x + round(pt_relative_x);
		int pt_y = pose.location.y + round(pt_relative_y);

		dst.data[3 * pt_y * dst.cols + 3 * pt_x] = 0;
		dst.data[3 * pt_y * dst.cols + 3 * pt_x + 1] = 0;
		dst.data[3 * pt_y * dst.cols + 3 * pt_x + 2] = 255;
	}
}

void VirtualWorld2D::GetSensorData(Rover2D *rover)
{
	Pose2D pose = rover->pose_last;

	rover->landmarks_observed.clear();

	for (const Pose2D& landmark : landmarks)
	{
		// ToDo : add error
		float dist_lanrmark = CalDistancePose(pose, landmark);

		if (dist_lanrmark >= info_rover.max_sensor_range)
			continue;

		// ToDo : add error
		float angle_landmark = CalAnglePose(pose, landmark) - pose.theta;
		float angle_landmark_count = AdjustAngle(angle_landmark);
		if (abs(angle_landmark_count) < info_rover.max_sensor_fov)
		{
			Pose2D landmark_observed;// = CreatePose(dist_lanrmark, angle_landmark);
			landmark_observed.location = landmark.location - pose.location;
			landmark_observed.theta = angle_landmark;
			rover->landmarks_observed.push_back(landmark_observed);
		}

	}
	

}

cv::Point2d  VirtualWorld2D::CalRotatedPt(Pose2D pose, int padding)
{
	Eigen::MatrixXf matrix_pt_rover_to_center = Eigen::MatrixXf(2, 1);
	matrix_pt_rover_to_center(0) = pose.location.x - padding;	// shfit rover location by padding, shift origin to image center
	matrix_pt_rover_to_center(1) = pose.location.y - padding;
	Eigen::MatrixXf matrix_rotate_90deg = create_rotation_matrix(90.0);
	Eigen::MatrixXf matrix_rotate_rover_theta = create_rotation_matrix(pose.theta);
	Eigen::MatrixXf matrix_pt_rover_rotate = matrix_pt_rover_to_center.transpose() * matrix_rotate_rover_theta * matrix_rotate_90deg;

	cv::Point2d pt_rover_rotate(round(matrix_pt_rover_rotate(0)), round(matrix_pt_rover_rotate(1)));

	return pt_rover_rotate;
}
