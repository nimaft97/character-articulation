#include "Jacobian.h"

Jacobian::Jacobian(glm::dvec4 point,
	double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, double theta7,
	double l1, double l2, double l3) :
	m_point(point),
	m_theta1(theta1), m_theta2(theta2), m_theta3(theta3), m_theta4(theta4), m_theta5(theta5), m_theta6(theta6), m_theta7(theta7),
	m_length_shoulder_to_elbow(l1), m_length_elbow_to_wrist(l2), m_length_wrist_to_finger(l3),
	m_jacobian(3, 7)
{
	// initialize transformation matrices
	m_transformation_root = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
	m_transformation_shoulder = {
			1, 0, 0, 0,
			0, 1, 0, -l1,
			0, 0, 1, 0,
			0, 0, 0, 1
	};
	m_transformation_elbow = {
		1, 0, 0, 0,
		0, 1, 0, -l2,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
	m_transformation_wrist = {
		1, 0, 0, 0,
		0, 1, 0, -l3,
		0, 0, 1, 0,
		0, 0, 0, 1
	};

	// populate Jacobian
	std::vector<glm::dvec3> jacobian_columns = {
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

glm::dvec4 Jacobian::RollX(double theta, glm::dvec4 point) {
	glm::dmat4x4 roll_matrix{
		1, 0, 0, 0,
		0, cos(theta), -sin(theta), 0,
		0, sin(theta), cos(theta), 0,
		0, 0, 0, 1
	};
	return roll_matrix * point;
}

glm::dvec4 Jacobian::RollY(double theta, glm::dvec4 point) {
	glm::dmat4x4 roll_matrix{
		cos(theta), 0, sin(theta), 1,
		0, 1, 0, 0,
		-sin(theta), 0, cos(theta), 0,
		0, 0, 0, 1
	};
	return roll_matrix * point;
}

glm::dvec4 Jacobian::RollZ(double theta, glm::dvec4 point) {
	glm::dmat4x4 roll_matrix{
		cos(theta), -sin(theta), 0, 0,
		sin(theta), cos(theta), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
	return roll_matrix * point;
}

glm::dvec4 Jacobian::RollXDerivative(double theta, glm::dvec4 point) {
	glm::dmat4x4 roll_matrix{
		1, 0, 0, 0,
		0, -sin(theta), -cos(theta), 0,
		0, cos(theta), -sin(theta), 0,
		0, 0, 0, 1
	};
	return roll_matrix * point;
}

glm::dvec4 Jacobian::RollYDerivative(double theta, glm::dvec4 point) {
	glm::dmat4x4 roll_matrix{
		-sin(theta), 0, cos(theta), 1,
		0, 1, 0, 0,
		-cos(theta), 0, -sin(theta), 0,
		0, 0, 0, 1
	};
	return roll_matrix * point;
}

glm::dvec4 Jacobian::RollZDerivative(double theta, glm::dvec4 point) {
	glm::dmat4x4 roll_matrix{
		-sin(theta), cos(theta), 0, 0,
		cos(theta), -sin(theta), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
	return roll_matrix * point;
}

glm::dvec3 Jacobian::DerivativeTheta1(glm::dvec4 point) {
	// initialization
	glm::dvec4 result = point;
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

	return glm::dvec3{ result[0], result[1], result[2] };
}

glm::dvec3 Jacobian::DerivativeTheta2(glm::dvec4 point) {
	// initialization
	glm::dvec4 result = point;
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

	return glm::dvec3{ result[0], result[1], result[2] };
}

glm::dvec3 Jacobian::DerivativeTheta3(glm::dvec4 point) {
	// initialization
	glm::dvec4 result = point;
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

	return glm::dvec3{ result[0], result[1], result[2] };
}

glm::dvec3 Jacobian::DerivativeTheta4(glm::dvec4 point) {
	// initialization
	glm::dvec4 result = point;
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

	return glm::dvec3{ result[0], result[1], result[2] };
}

glm::dvec3 Jacobian::DerivativeTheta5(glm::dvec4 point) {
	// initialization
	glm::dvec4 result = point;
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

	return glm::dvec3{ result[0], result[1], result[2] };
}

glm::dvec3 Jacobian::DerivativeTheta6(glm::dvec4 point) {
	// initialization
	glm::dvec4 result = point;
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

	return glm::dvec3{ result[0], result[1], result[2] };
}

glm::dvec3 Jacobian::DerivativeTheta7(glm::dvec4 point) {
	// initialization
	glm::dvec4 result = point;
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

	return glm::dvec3{ result[0], result[1], result[2] };
}