#include "util.h"

float AdjustAngle(float theta)
{
	if (theta > 180.0)
		theta -= 360;
	else if (theta < -180.0)
		theta += 360;

	return theta;
};

float CalAnglePose(Pose2D a, Pose2D b)
{
	//a기준 b가 어느방향에 있는가?
	float diff_x = b.location.x - a.location.x;
	float diff_y = b.location.y - a.location.y;

	float angle_in_field = atan2(diff_y, diff_x);
	float angle_in_field_deg = Rad2Deg(angle_in_field);

	angle_in_field_deg = AdjustAngle(angle_in_field_deg);

	return angle_in_field_deg;

}

float CalDistancePose(Pose2D a, Pose2D b)
{
	float diff_x = a.location.x - b.location.x;
	float diff_y = a.location.y - b.location.y;

	float dist = sqrt(diff_x * diff_x + diff_y * diff_y);

	return dist;

};

float Deg2Rad(float degree)
{
	return (degree * PI / 180.0);
};

float Rad2Deg(float radian)
{
	return (radian * 180.0 / PI);
}
Pose2D CreatePose(float range, float theta)
{
	Pose2D tmp_pose;
	tmp_pose.location = cv::Point2d(round(range * cos(Deg2Rad(theta))), round(range * sin(Deg2Rad(theta))));
	tmp_pose.theta = theta;
	return tmp_pose;
}

Pose2D CreatePose(cv::Point2d location, float theta)
{
	Pose2D tmp_pose;
	tmp_pose.location = location;
	tmp_pose.theta = theta;
	return tmp_pose;
}

float CalRange(Pose2D pose)
{
	return sqrt(pose.location.x * pose.location.x + pose.location.y * pose.location.y);
}

Eigen::MatrixXf create_rotation_matrix(float rotate_deg)
{
	float rotate_rad = rotate_deg * PI / 180.0;

	Eigen::MatrixXf created_rotation = Eigen::MatrixXf(2, 2);
	created_rotation(0, 0) = cos(rotate_rad);
	created_rotation(0, 1) = -sin(rotate_rad);
	created_rotation(1, 0) = sin(rotate_rad);
	created_rotation(1, 1) = cos(rotate_rad);

	return created_rotation;
};
