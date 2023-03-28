#include "Transformation.h"
#include "utils.h"

double Transformation::degree2radian(double d)
{
	return d * 3.141592653589 / 180.0;
}

Transformation::Transformation(
	double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, double theta7,
	double l1, double l2, double l3) :
	m_theta1(degree2radian(theta1)), m_theta2(degree2radian(theta2)), m_theta3(degree2radian(theta3)),
	m_theta4(degree2radian(theta4)), m_theta5(degree2radian(theta5)), m_theta6(degree2radian(theta6)),
	m_theta7(degree2radian(theta7)),
	m_length_shoulder_to_elbow(l1), m_length_elbow_to_wrist(l2), m_length_wrist_to_finger(l3)
{

}

Eigen::Vector4d Transformation::RollX(double theta, const Eigen::Vector4d& point) {
	Eigen::Matrix4d roll_matrix;
	roll_matrix <<
		1, 0, 0, 0,
		0, cos(theta), -sin(theta), 0,
		0, sin(theta), cos(theta), 0,
		0, 0, 0, 1
		;
	return roll_matrix * point;
}

Eigen::Vector4d Transformation::RollY(double theta, const Eigen::Vector4d& point) {
	Eigen::Matrix4d roll_matrix;
	roll_matrix <<
		cos(theta), 0, sin(theta), 0,
		0, 1, 0, 0,
		-sin(theta), 0, cos(theta), 0,
		0, 0, 0, 1
		;
	return roll_matrix * point;
}

Eigen::Vector4d Transformation::RollZ(double theta, const Eigen::Vector4d& point) {
	Eigen::Matrix4d roll_matrix;
	roll_matrix <<
		cos(theta), -sin(theta), 0, 0,
		sin(theta), cos(theta), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
		;
	return roll_matrix * point;
}

Eigen::Vector4d Transformation::TranslateY(double y, const Eigen::Vector4d& point) {
	Eigen::Matrix4d translation_matrix;
	translation_matrix <<
		1, 0, 0, 0,
		0, 1, 0, y,
		0, 0, 1, 0,
		0, 0, 0, 1
		;
	return translation_matrix * point;
}

Eigen::Vector3d Transformation::transform()
{
	// torso_width / 2.0, torso_height / 2.0, z_distance
	Eigen::Vector4d ret;
	ret << 0.6, 1.5, 3.0, 1.0; // where shoulder starts
	
	ret = RollZ(m_theta3, ret);
	ret = RollY(m_theta2, ret);
	ret = RollX(m_theta1, ret);
	ret = TranslateY(-m_length_shoulder_to_elbow, ret);

	ret = RollX(m_theta4, ret);
	ret = RollY(m_theta5, ret);
	ret = TranslateY(-m_length_elbow_to_wrist, ret);

	ret = RollX(m_theta6, ret);
	ret = RollY(m_theta7, ret);
	ret = TranslateY(-m_length_wrist_to_finger, ret);
	
	
	/*
	ret = RollX(m_theta6, ret);
	ret = RollY(m_theta7, ret);
	ret = TranslateY(-m_length_wrist_to_finger, ret);

	ret = RollX(m_theta4, ret);
	ret = RollY(m_theta5, ret);
	ret = TranslateY(-m_length_elbow_to_wrist, ret);

	ret = RollZ(m_theta3, ret);
	ret = RollY(m_theta2, ret);
	ret = RollX(m_theta1, ret);
	ret = TranslateY(-m_length_shoulder_to_elbow, ret);
	*/

	return ret.head(3);
}