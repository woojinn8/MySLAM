#pragma once
#include "util.h"
#include "Rover.h"
#include <fstream>

class VirtualWorld2D {

public:
	void Initialize(std::string path_map_img, std::string path_waypoint_txt);

	void run();


	std::vector<Pose2D> landmarks;
	std::vector<Pose2D> waypoints;

	cv::Mat map_img;

	Rover2D rover_inworld, rover_inner;

	RoverSpec info_rover;

private:

	void LoadMapfromImage(std::string path_map_img);
	void LoadWaypointfromTxt(std::string path_waypoint_txt);

	void DisplayMap();
	void Display_DrawRover(cv::Mat map_display, Pose2D pose, cv::Scalar color);

	void Display_RotateMap(cv::Mat src, cv::Mat *dst, Pose2D pose);
	void Display_CropRoverView(cv::Mat src, cv::Mat *dst, Pose2D pose);

	void Display_DrawLandmark(cv::Mat dst, Pose2D pose, float theta_basis, std::vector<Pose2D> landmarks);

	void GetSensorData(Rover2D *pose);

	cv::Point2d CalRotatedPt(Pose2D pose, int padding);
	
};