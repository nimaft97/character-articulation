#include "Jacobian.h"
#include "utils.h"

double Jacobian::degree2radian(double d)
{
	return d * 3.14159265358979323846 / 180.0;
}

Jacobian::Jacobian(Eigen::Vector4d point,
	double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, double theta7,
	double l1, double l2, double l3) :
	m_point(point),
	m_theta1(degree2radian(theta1)), m_theta2(degree2radian(theta2)), m_theta3(degree2radian(theta3)), 
	m_theta4(degree2radian(theta4)), m_theta5(degree2radian(theta5)), m_theta6(degree2radian(theta6)), 
	m_theta7(degree2radian(theta7)),
	m_length_shoulder_to_elbow(l1), m_length_elbow_to_wrist(l2), m_length_wrist_to_finger(l3),
	m_jacobian(3, 7)
{
	// initialize transformation matrices
	m_transformation_root <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	;
	m_transformation_shoulder <<
			1, 0, 0, 0,
			0, 1, 0, -l1,
			0, 0, 1, 0,
			0, 0, 0, 1
	;
	m_transformation_elbow <<
		1, 0, 0, 0,
		0, 1, 0, -l2,
		0, 0, 1, 0,
		0, 0, 0, 1
	;
	m_transformation_wrist <<
		1, 0, 0, 0,
		0, 1, 0, -l3,
		0, 0, 1, 0,
		0, 0, 0, 1
	;

	// populate Jacobian
	std::vector<Eigen::Vector3d> jacobian_columns = {
		DerivativeTheta1(m_point), DerivativeTheta2(m_point), DerivativeTheta3(m_point), DerivativeTheta4(m_point),
		DerivativeTheta5(m_point), DerivativeTheta6(m_point), DerivativeTheta7(m_point)
	};
	m_jacobian <<
		jacobian_columns[0][0], jacobian_columns[1][0], jacobian_columns[2][0], jacobian_columns[3][0],
		jacobian_columns[4][0], jacobian_columns[5][0], jacobian_columns[6][0],

		jacobian_columns[0][1], jacobian_columns[1][1], jacobian_columns[2][1], jacobian_columns[3][1],
		jacobian_columns[4][1], jacobian_columns[5][1], jacobian_columns[6][1],

		jacobian_columns[0][2], jacobian_columns[1][2], jacobian_columns[2][2], jacobian_columns[3][2],
		jacobian_columns[4][2], jacobian_columns[5][2], jacobian_columns[6][2];
}

Eigen::Vector4d Jacobian::RollX(double theta, const Eigen::Vector4d& point) {
	Eigen::Matrix4d roll_matrix;
	roll_matrix <<
		1, 0, 0, 0,
		0, cos(theta), -sin(theta), 0,
		0, sin(theta), cos(theta), 0,
		0, 0, 0, 1
	;
	return roll_matrix * point;
}

Eigen::Vector4d Jacobian::RollY(double theta, const Eigen::Vector4d& point) {
	Eigen::Matrix4d roll_matrix;
	roll_matrix <<
		cos(theta), 0, sin(theta), 0,
		0, 1, 0, 0,
		-sin(theta), 0, cos(theta), 0,
		0, 0, 0, 1
	;
	return roll_matrix * point;
}

Eigen::Vector4d Jacobian::RollZ(double theta, const Eigen::Vector4d& point) {
	Eigen::Matrix4d roll_matrix;
	roll_matrix <<
		cos(theta), -sin(theta), 0, 0,
		sin(theta), cos(theta), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	;
	return roll_matrix * point;
}

Eigen::Vector4d Jacobian::RollXDerivative(double theta, const Eigen::Vector4d& point) {
	Eigen::Matrix4d roll_matrix;
	roll_matrix <<
		0, 0, 0, 0,
		0, -sin(theta), -cos(theta), 0,
		0, cos(theta), -sin(theta), 0,
		0, 0, 0, 0
	;
	return roll_matrix * point;
}

Eigen::Vector4d Jacobian::RollYDerivative(double theta, const Eigen::Vector4d& point) {
	Eigen::Matrix4d roll_matrix;
	roll_matrix <<
		-sin(theta), 0, cos(theta), 0,
		0, 0, 0, 0,
		-cos(theta), 0, -sin(theta), 0,
		0, 0, 0, 0
	;
	return roll_matrix * point;
}

Eigen::Vector4d Jacobian::RollZDerivative(double theta, const Eigen::Vector4d& point) {
	Eigen::Matrix4d roll_matrix;
	roll_matrix <<
		-sin(theta), cos(theta), 0, 0,
		cos(theta), -sin(theta), 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0
	;
	return roll_matrix * point;
}

Eigen::Vector3d Jacobian::DerivativeTheta1(const Eigen::Vector4d& point) {
	// initialization
	Eigen::Vector4d result = point;
	// equation
	result = RollX(m_theta6, result);
	result = RollY(m_theta7, result);
	result = m_transformation_wrist * result;

	result = RollX(m_theta4, result);
	result = RollY(m_theta5, result);
	result = m_transformation_elbow * result;

	result = RollXDerivative(m_theta1, result);
	result = RollY(m_theta2, result);
	result = RollZ(m_theta3, result);

	result = m_transformation_root * result;
	
	return result.head(3);
}

Eigen::Vector3d Jacobian::DerivativeTheta2(const Eigen::Vector4d& point) {
	// initialization
	Eigen::Vector4d result = point;
	// equation
	result = RollX(m_theta6, result);
	result = RollY(m_theta7, result);
	result = m_transformation_wrist * result;

	result = RollX(m_theta4, result);
	result = RollY(m_theta5, result);
	result = m_transformation_elbow * result;

	result = RollX(m_theta1, result);
	result = RollYDerivative(m_theta2, result);
	result = RollZ(m_theta3, result);

	result = m_transformation_root * result;

	return result.head(3);
}

Eigen::Vector3d Jacobian::DerivativeTheta3(const Eigen::Vector4d& point) {
	// initialization
	Eigen::Vector4d result = point;
	// equation
	result = RollX(m_theta6, result);
	result = RollY(m_theta7, result);
	result = m_transformation_wrist * result;

	result = RollX(m_theta4, result);
	result = RollY(m_theta5, result);
	result = m_transformation_elbow * result;

	result = RollX(m_theta1, result);
	result = RollY(m_theta2, result);
	result = RollZDerivative(m_theta3, result);

	result = m_transformation_root * result;

	return result.head(3);
}

Eigen::Vector3d Jacobian::DerivativeTheta4(const Eigen::Vector4d& point) {
	// initialization
	Eigen::Vector4d result = point;
	// equation
	result = RollX(m_theta6, result);
	result = RollY(m_theta7, result);
	result = m_transformation_wrist * result;

	result = RollXDerivative(m_theta4, result);
	result = RollY(m_theta5, result);
	result = m_transformation_elbow * result;

	result = RollX(m_theta1, result);
	result = RollY(m_theta2, result);
	result = RollZ(m_theta3, result);

	result = m_transformation_root * result;

	return result.head(3);
}

Eigen::Vector3d Jacobian::DerivativeTheta5(const Eigen::Vector4d& point) {
	// initialization
	Eigen::Vector4d result = point;
	// equation
	result = RollX(m_theta6, result);
	result = RollY(m_theta7, result);
	result = m_transformation_wrist * result;

	result = RollX(m_theta4, result);
	result = RollYDerivative(m_theta5, result);
	result = m_transformation_elbow * result;

	result = RollX(m_theta1, result);
	result = RollY(m_theta2, result);
	result = RollZ(m_theta3, result);

	result = m_transformation_root * result;

	return result.head(3);
}

Eigen::Vector3d Jacobian::DerivativeTheta6(const Eigen::Vector4d& point) {
	// initialization
	Eigen::Vector4d result = point;
	// equation
	result = RollXDerivative(m_theta6, result);
	result = RollY(m_theta7, result);
	result = m_transformation_wrist * result;

	result = RollX(m_theta4, result);
	result = RollY(m_theta5, result);
	result = m_transformation_elbow * result;

	result = RollX(m_theta1, result);
	result = RollY(m_theta2, result);
	result = RollZ(m_theta3, result);

	result = m_transformation_root * result;

	return result.head(3);
}

Eigen::Vector3d Jacobian::DerivativeTheta7(const Eigen::Vector4d& point) {
	// initialization
	Eigen::Vector4d result = point;
	// equation
	result = RollX(m_theta6, result);
	result = RollYDerivative(m_theta7, result);
	result = m_transformation_wrist * result;

	result = RollX(m_theta4, result);
	result = RollY(m_theta5, result);
	result = m_transformation_elbow * result;

	result = RollX(m_theta1, result);
	result = RollY(m_theta2, result);
	result = RollZ(m_theta3, result);

	result = m_transformation_root * result;

	return result.head(3);
}