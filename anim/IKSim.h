#ifndef IK_SIM_H
#define IK_SIM_H

#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "HermiteSystem.h"
#include "BobSystem.h"
#include "glm/glm.hpp"

#include <string>

// a sample simulator

class IKSim : public BaseSimulator 
{
public:

	IKSim( const std::string& name, BobSystem* target );
	~IKSim();

	int step(double time);
	int init(double time) 
	{ 
		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);

private:
	void InitializePoints();
	glm::dvec3 NextPoint(int idx, double t);

private:
	double m_epsilon = 0.001;
	glm::dvec3 m_point_current;
	glm::dvec3 m_point_target;
	double m_spline_delta_time;
	double m_k = 0.1;
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

protected:
	BobSystem* m_object;


};


#endif