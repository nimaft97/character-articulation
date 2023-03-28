#include "Solver.h"

Solver::Solver(Eigen::MatrixXd jacobian, Eigen::Vector3d dx): m_jacobian(jacobian), m_dx(dx) {

}

Eigen::Vector3d Solver::SolveBeta()
{
	// Ax = b
	Eigen::Vector3d b; // velocity
	Eigen::Matrix3d A; // JJT
	b << m_dx[0], m_dx[1], m_dx[2];
	A = m_jacobian * m_jacobian.transpose();
	Eigen::PartialPivLU<Eigen::Matrix3d> lu(A);
	return lu.solve(b); // beta 3x1
}

std::vector<double> Solver::CalculateDeltaTheta()
{
	// 7x3 * 3x1 = 7x1
	auto result = m_jacobian.transpose() * SolveBeta() * (180.0/ 3.14159265358979323846); // conversion from radian to degree is required
	return { result[0], result[1], result[2], result[3], result[4], result[5], result[6] };
}