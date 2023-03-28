#include "Jacobian.h"
#include "Solver.h"
#include "IKSim.h"
#include "Transformation.h"

IKSim::IKSim( const std::string& name, BobSystem* target ):
	BaseSimulator( name ),
	m_object( target )
{
}	// SampleGravitySimulator

IKSim::~IKSim()
{
}	// SampleGravitySimulator::~SampleGravitySimulator

void IKSim::UpdateThetas(const std::vector<double>& delta_theta)
{
	// detla_theta is a 7x1 vector
	m_object->m_theta1 += delta_theta[0];
	m_object->m_theta2 += delta_theta[1];
	m_object->m_theta3 += delta_theta[2];
	m_object->m_theta4 += delta_theta[3];
	m_object->m_theta5 += delta_theta[4];
	m_object->m_theta6 += delta_theta[5];
	m_object->m_theta7 += delta_theta[6];
}

// whenever the step function is called for the first time, 
// current point is the first point of the spline 
// and the target point is the second control point
int IKSim::step(double time)
{
	static int idx = 0;
	static double t = m_spline_delta_time;
	if (t <= 1.0001 && t >= 0.9999)
	{
		t = 0.0;
		idx++;
	}
	if (idx == m_object->hermiteSystem->numPoints - 1)
	{
		return 0;
	}
	// update points
	t += m_spline_delta_time;
	// m_point current is initially set to the first point of the spline
	m_point_target = NextPoint(idx, t);
	// E = Ptarrget - Pcurrent
	Eigen::Vector3d E = m_point_target - m_point_current;
	while (E.norm() > m_epsilon)
	{
		auto dx = m_k * E;
		Eigen::Vector4d input_point;
		input_point << m_point_current[0], m_point_current[1], m_point_current[2], 1.0;
		Jacobian jacobian = Jacobian(input_point,
			m_object->m_theta1, m_object->m_theta2, m_object->m_theta3, 
			m_object->m_theta4, m_object->m_theta5, m_object->m_theta6, m_object->m_theta7,
			m_object->l1_length, m_object->l2_length, m_object->l3_length);

		// uses the pseudo inverse to solve for beta and calculate delta thetas
		Solver solver = Solver(jacobian.m_jacobian, dx);
		const auto& delta_theta = solver.CalculateDeltaTheta();
		// update thetas: new theta = current theta + delta theta
		UpdateThetas(delta_theta);
		// move the current point
		m_point_current += dx;
		break; // :)
	}
	// update the current point
	m_point_current = m_point_target;


	return 0;

}	// SampleGravitySimulator::step

int IKSim::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "read") == 0)
	{
		if (argc == 2)
		{
			m_object->hermiteSystem->command(argc, argv);
			m_object->m_spline_is_loaded = true;
			// now that spline is loaded, initialize points
			InitializePoints();
		}
		else
		{
			animTcl::OutputMessage("Usage: read <file_name>");
			return TCL_ERROR;
		}
	}
	
	glutPostRedisplay();
	return TCL_OK;
}

void IKSim::InitializePoints()
{
	/*
	HermiteSplineSystem* hermite = m_object->hermiteSystem;
	m_spline_delta_time = hermite->deltaT;
	m_point_current = NextPoint(0, 0.0);
	*/
	HermiteSplineSystem* hermite = m_object->hermiteSystem;
	m_spline_delta_time = hermite->deltaT;
	Transformation transformation(m_object->m_theta1, m_object->m_theta2, m_object->m_theta3, m_object->m_theta4,
		m_object->m_theta5, m_object->m_theta6, m_object->m_theta7,
		m_object->l1_length, m_object->l2_length, m_object->l3_length);
	m_point_current = transformation.transform();
	// test
	m_object->test1 = m_point_current[0];
	m_object->test2 = m_point_current[1];
	m_object->test3 = m_point_current[2];
}

// this function returns one intermediate point from the spline
Eigen::Vector3d IKSim::NextPoint(int idx, double t)
{
	HermiteSplineSystem* hermite = m_object->hermiteSystem;
	Eigen::Vector3d ret;
	double x, y, z;
	hermite->nextPoint(t, idx, x, y, z);
	ret << x, y, z;
	return ret;
}
