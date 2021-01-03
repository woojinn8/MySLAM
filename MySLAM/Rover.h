#pragma once
#include "util.h"
#include "GraphBasedSLAM.h"

class Rover2D {

private:
	GraphBasedSLAM optimize;


public:


	
	void Initialize(Pose2D pose_init);

	void MoveNextWaypoint(Pose2D waypoint);
	void MoveAccordingtoControl(Pose2D control, Pose2D error);

	std::vector<Pose2D> pose_trajectory;
	Pose2D pose_last;

	std::vector<Pose2D> control_history;
	Pose2D control_last;

	std::vector<Pose2D> landmarks_observed;

	RoverSpec info_rover;

private:
	Pose2D GetLastPose();
	Pose2D GetLastControl();




};