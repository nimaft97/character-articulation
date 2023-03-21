#ifndef SOLVER_H
#define SOLVER_H

#include "Jacobian.h"

struct Solver {
public:
	Solver(Eigen::MatrixXd jacobian, glm::dvec3 dx);
private:


public:

private:
	glm::dvec3 m_dx;
	Eigen::MatrixXd m_jacobian;
};


#endif