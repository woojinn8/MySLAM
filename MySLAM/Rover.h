#pragma once
#include "util.h"

class Rover2D {

public:

	void Initialize(Pose2D pose_init);
	void MoveNextWaypoint(Pose2D waypoint);

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