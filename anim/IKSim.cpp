#include "Jacobian.h"
#include "IKSim.h"

IKSim::IKSim( const std::string& name, BobSystem* target ):
	BaseSimulator( name ),
	m_object( target )
{
}	// SampleGravitySimulator

IKSim::~IKSim()
{
}	// SampleGravitySimulator::~SampleGravitySimulator

// whenever the step function is called for the first time, 
// current point is the first point of the spline 
// and the target point is the second control point
int IKSim::step(double time)
{
	static int idx = 0;
	static double t = m_spline_delta_time;
	if (t == 1.0)
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
	m_point_current = m_point_target;
	m_point_target = NextPoint(idx, t);
	// E = Ptarrget - Pcurrent
	glm::dvec3 E = m_point_target - m_point_current;
	while (glm::length(E) > m_epsilon) 
	{
		glm::dvec3 dx = m_k * E;
		Jacobian* jacobian = new Jacobian(glm::dvec4{m_point_current, 1.0},
			m_theta1, m_theta2, m_theta3, m_theta4, m_theta5, m_theta6, m_theta7,
			m_object->l1_length, m_object->l2_length, m_object->l3_length);
	}

	// update current and target points


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
	HermiteSplineSystem* hermite = m_object->hermiteSystem;
	m_spline_delta_time = hermite->deltaT;
	m_point_current = NextPoint(0, 0.0);
}

// this function returns one intermediate point from the spline
glm::dvec3 IKSim::NextPoint(int idx, double t)
{
	HermiteSplineSystem* hermite = m_object->hermiteSystem;
	glm::dvec3 ret(0.0, 0.0, 0.0);
	hermite->nextPoint(t, idx, ret[0], ret[1], ret[2]);
	return ret;
}
