#include "Rover.h"

void Rover2D::Initialize(Pose2D pose_init)
{
	pose_last = pose_init;
	pose_trajectory.push_back(pose_last);
}


void Rover2D::MoveNextWaypoint(Pose2D waypoint)
{
	pose_last = GetLastPose();

	// check direction and distance
	float dist_waypoint = CalDistancePose(pose_last, waypoint);
	float angle_waypoint = CalAnglePose(pose_last, waypoint);

	// take plan to move (decide control)
		// 1. rotate to next waypoint direction
		// 2. move to next waypoint

	float count_theta = AdjustAngle(angle_waypoint - pose_last.theta);

	int rotate_direction = 0;
	if (count_theta > 0)
		rotate_direction = 1;
	else
		rotate_direction = -1;

	float moving_range;
	float moving_theta;
	if (abs(count_theta) > info_rover.max_rotation_per_cycle)
	{
		moving_theta = rotate_direction * info_rover.max_rotation_per_cycle;
		moving_range = 0.0;
	}
	else
	{
		moving_theta = count_theta;

		if (dist_waypoint > info_rover.max_speed_per_cycle)
			moving_range = info_rover.max_speed_per_cycle;
		else
			moving_range = dist_waypoint;
	}

	Pose2D control = CreatePose(moving_range, moving_theta);

	MoveAccordingtoControl(control);

}

void Rover2D::MoveAccordingtoControl(Pose2D control)
{
	// set control (direction & moving distance)
	float range = CalRange(control);
	float theta = Deg2Rad(pose_last.theta + control.theta);

	// move
	cv::Point2d moving_point;
	moving_point.x = pose_last.location.x + round(range * cos(theta));
	moving_point.y = pose_last.location.y + round(range * sin(theta));
	Pose2D moved_pose;
	float updated_deg = AdjustAngle(pose_last.theta + control.theta);
	moved_pose = CreatePose(moving_point, updated_deg);

	// update information
	pose_trajectory.push_back(moved_pose);
	control_history.push_back(control);

	pose_last = GetLastPose();
	control_last = GetLastControl();

}

Pose2D Rover2D::GetLastPose()
{
	return pose_trajectory.back();
}

Pose2D Rover2D::GetLastControl()
{
	return control_history.back();
}
