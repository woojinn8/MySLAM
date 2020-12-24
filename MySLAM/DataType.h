#pragma once

#include <opencv2/opencv.hpp>

struct Pose2D {
	cv::Point2d location;	// x, y position
	float theta;			// degree
};

const struct RoverSpec {

	const float max_speed_per_cycle = 10;
	const float max_rotation_per_cycle = 30;

	const float max_sensor_range = 100;
	const float max_sensor_fov = 60;

	const float maxerror_sensor_range = 3;
	const float maxerror_sensor_angle = 1;
	const float maxerror_moving_range = 1;
	const float maxerror_moving_angle = 3;
};