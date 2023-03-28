#ifndef JACOBIAN_H
#define JACOBIAN_H

#include "Eigen/Dense"
#include "glm/glm.hpp"
#include <vector>

struct Jacobian {
public:
	Jacobian(Eigen::Vector4d point,
		double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, double theta7,
		double l1, double l2, double l3);

private:
	Eigen::Vector4d RollX(double theta, const Eigen::Vector4d& point); // rotation along x
	Eigen::Vector4d RollY(double theta, const Eigen::Vector4d& point); // rotation along y
	Eigen::Vector4d RollZ(double theta, const Eigen::Vector4d& point); // rotation along z

	Eigen::Vector4d RollXDerivative(double theta, const Eigen::Vector4d& point); // derivative of rotation along x
	Eigen::Vector4d RollYDerivative(double theta, const Eigen::Vector4d& point); // derivative of rotation along y
	Eigen::Vector4d RollZDerivative(double theta, const Eigen::Vector4d& point); // derivative of rotation along z

	Eigen::Vector3d DerivativeTheta1(const Eigen::Vector4d& point); // first column of Jacobian matrix (3x1)
	Eigen::Vector3d DerivativeTheta2(const Eigen::Vector4d& point); // second column of Jacobian matrix (3x1)
	Eigen::Vector3d DerivativeTheta3(const Eigen::Vector4d& point); // third column of Jacobian matrix (3x1)
	Eigen::Vector3d DerivativeTheta4(const Eigen::Vector4d& point); // fourth column of Jacobian matrix (3x1)
	Eigen::Vector3d DerivativeTheta5(const Eigen::Vector4d& point); // fifth column of Jacobian matrix (3x1)
	Eigen::Vector3d DerivativeTheta6(const Eigen::Vector4d& point); // sixth column of Jacobian matrix (3x1)
	Eigen::Vector3d DerivativeTheta7(const Eigen::Vector4d& point); // seventh column of Jacobian matrix (3x1)

	double degree2radian(double d);

public:
	Eigen::MatrixXd m_jacobian; // 3 x 7
	
private:
	// point
	Eigen::Vector4d m_point;
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
	Eigen::Matrix4d m_transformation_root;
	Eigen::Matrix4d m_transformation_shoulder;
	Eigen::Matrix4d m_transformation_elbow;
	Eigen::Matrix4d m_transformation_wrist;
};

#endif