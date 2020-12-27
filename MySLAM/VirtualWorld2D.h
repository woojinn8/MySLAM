#pragma once
#include "util.h"
#include "Rover.h"
#include <fstream>


// Define Color

#define Color_RED CV_RGB(255, 0, 0)
#define Color_BLUE CV_RGB(0, 0, 255)
#define Color_ORANGE CV_RGB(255, 165, 0)
#define Color_GREEN CV_RGB(0, 255, 0)
#define Color_PINK CV_RGB(255, 133, 255)

#define Color_waypoint Color_BLUE

#define Color_rover_inworld Color_GREEN
#define Color_landmark_inworld Color_RED

#define Color_rover_inner Color_ORANGE
#define Color_landmark_inner Color_PINK

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

	// Initialize
	void LoadMapfromImage(std::string path_map_img);
	void LoadWaypointfromTxt(std::string path_waypoint_txt);

	// Display
	void DisplayMap();
	
		// Display info to map
	void Display_DrawRover(cv::Mat map_display, Pose2D pose, cv::Scalar color);
	void Display_DrawTrajectory(cv::Mat map_display, std::vector<Pose2D> trajectory, cv::Scalar color);

	void Display_RotateMap(cv::Mat src, cv::Mat *dst, Pose2D pose);
	void Display_CropRoverView(cv::Mat src, cv::Mat *dst, Pose2D pose);

	void Display_DrawLandmark(cv::Mat dst, Pose2D pose, float theta_basis, std::vector<Pose2D> landmarks, cv::Scalar color);


	// Rover's Sensor data
	void GetSensorData(Rover2D *rover);
	void AddErrortoSensor(Rover2D* rover);
	void AddErrortoActuator(Pose2D* control);

	cv::Point2d CalRotatedPt(Pose2D pose, float angle_rotate, int padding);
	
	


};