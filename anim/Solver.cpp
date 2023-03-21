#include "Solver.h"

Solver::Solver(Eigen::MatrixXd jacobian, glm::dvec3 dx): m_jacobian(jacobian), m_dx(dx) {

}
