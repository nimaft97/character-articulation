#ifndef IK_SIM_H
#define IK_SIM_H

#include "Eigen/Dense"
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
	Eigen::Vector3d NextPoint(int idx, double t);
	void UpdateThetas(const std::vector<double>& delta_theta);

private:
	double m_epsilon = 0.001;
	Eigen::Vector3d m_point_current;
	Eigen::Vector3d m_point_target;
	double m_spline_delta_time;
	double m_k = 1.0; // since we are using intermediate points from the hermite spline which are close enough

protected:
	BobSystem* m_object;


};


#endif