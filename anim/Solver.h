#ifndef SOLVER_H
#define SOLVER_H

#include "Jacobian.h"

struct Solver {
public:
	Solver(Eigen::MatrixXd jacobian, Eigen::Vector3d dx);
	std::vector<double> CalculateDeltaTheta();
private:
	Eigen::Vector3d SolveBeta();

public:

private:
	Eigen::VectorXd m_dx;
	Eigen::MatrixXd m_jacobian;
};


#endif