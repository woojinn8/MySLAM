#pragma once
#include "DataType.h"
#include <Eigen/Dense>

#define PI 3.1415926535897


float AdjustAngle(float theta);

float CalAnglePose(Pose2D a, Pose2D b);

float CalDistancePose(Pose2D a, Pose2D b);

float Deg2Rad(float degree);

float Rad2Deg(float radian);

Pose2D CreatePose(float range, float theta);

Pose2D CreatePose(cv::Point2d location, float theta);

float CalRange(Pose2D pose);

Eigen::MatrixXf create_rotation_matrix(float rotate_deg);