#ifndef JACOBIAN_H
#define JACOBIAN_H

#include "Eigen/Dense"
#include "glm/glm.hpp"
#include <vector>

struct Jacobian {
public:
	Jacobian(glm::dvec4 point,
		double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, double theta7,
		double l1, double l2, double l3);

private:
	glm::dvec4 RollX(double theta, glm::dvec4 point); // rotation along x
	glm::dvec4 RollY(double theta, glm::dvec4 point); // rotation along y
	glm::dvec4 RollZ(double theta, glm::dvec4 point); // rotation along z

	glm::dvec4 RollXDerivative(double theta, glm::dvec4 point); // derivative of rotation along x
	glm::dvec4 RollYDerivative(double theta, glm::dvec4 point); // derivative of rotation along y
	glm::dvec4 RollZDerivative(double theta, glm::dvec4 point); // derivative of rotation along z

	glm::dvec3 DerivativeTheta1(glm::dvec4 point); // first column of Jacobian matrix (3x1)
	glm::dvec3 DerivativeTheta2(glm::dvec4 point); // second column of Jacobian matrix (3x1)
	glm::dvec3 DerivativeTheta3(glm::dvec4 point); // third column of Jacobian matrix (3x1)
	glm::dvec3 DerivativeTheta4(glm::dvec4 point); // fourth column of Jacobian matrix (3x1)
	glm::dvec3 DerivativeTheta5(glm::dvec4 point); // fifth column of Jacobian matrix (3x1)
	glm::dvec3 DerivativeTheta6(glm::dvec4 point); // sixth column of Jacobian matrix (3x1)
	glm::dvec3 DerivativeTheta7(glm::dvec4 point); // seventh column of Jacobian matrix (3x1)

public:
	Eigen::MatrixXd m_jacobian; // 3 x 7
	
private:
	// point
	glm::dvec4 m_point;
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
	// translation matrices
	glm::dmat4x4 m_transformation_root;
	glm::dmat4x4 m_transformation_shoulder;
	glm::dmat4x4 m_transformation_elbow;
	glm::dmat4x4 m_transformation_wrist;
};

#endif