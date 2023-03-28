#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include "Eigen/Dense"
#include "glm/glm.hpp"
#include <vector>

struct Transformation {
public:
	Transformation(
		double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, double theta7,
		double l1, double l2, double l3);
	Eigen::Vector3d transform();

private:
	Eigen::Vector4d RollX(double theta, const Eigen::Vector4d& point); // rotation along x
	Eigen::Vector4d RollY(double theta, const Eigen::Vector4d& point); // rotation along y
	Eigen::Vector4d RollZ(double theta, const Eigen::Vector4d& point); // rotation along z
	Eigen::Vector4d TranslateY(double y, const Eigen::Vector4d& point);

	double degree2radian(double d);

public:

private:
	// shoulder
	double m_theta1;
	double m_theta2;
	double m_theta3;
	// elbow
	double m_theta4;
	double m_theta5;
	// wrist
	double m_theta6;
	double m_theta7;
	// length of components of arm
	double m_length_shoulder_to_elbow;
	double m_length_elbow_to_wrist;
	double m_length_wrist_to_finger;
};

#endif