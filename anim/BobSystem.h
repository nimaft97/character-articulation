#ifndef BOB_SYSTEM_H
#define BOB_SYSTEM_H

/*

	This is a sample system. It accepts the command "read" followed by the 
	path to an OBJ model.

*/


#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "HermiteSystem.h"
#include <GLmodel/GLmodel.h>

#include "shared/opengl.h"

// a sample system
class BobSystem : public BaseSystem
{ 

public:
	BobSystem( const std::string& name );
	virtual void getState( double *p );
	virtual void setState( double  *p );
	void reset( double time );

	void displayClassroom(GLenum mode = GL_RENDER);
	void displayFixedPartsOfBody(GLenum mode = GL_RENDER);
	void displayRightHand(GLenum mode = GL_RENDER);
	void displayDot(double x, double y, double z);
	void display( GLenum mode = GL_RENDER );

	void readModel(char *fname) { m_model.ReadOBJ(fname); }
	void flipNormals(void) { glmReverseWinding(&m_model); }
	int command(int argc, myCONST_SPEC char **argv);

private:
	void InitializeThetas();

public:
	HermiteSplineSystem* hermiteSystem = new HermiteSplineSystem("hermite");
	bool m_spline_is_loaded = false;
	double l1_length = 2.0, l2_length = 2.0, l3_length = 0.5; // arm consists of L1 (shoulder to elbow), L2 (elbow to wrist), and L3 (wrist to finger)
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
	// test
	double test1, test2, test3;
protected:

	// define parameters for drawings
	double width_coefficient = 0.4;
	double blackboard_width = 12.0, blackboard_height = 8.0;
	double z_distance = 3.0;
	double l1_width = width_coefficient * l1_length, l2_width = width_coefficient * l2_length, l3_width = width_coefficient * l3_length;
	double wall_height = 12, wall_width = 24;
	double floor_height = wall_height, floor_width = wall_width;
	double torso_height = wall_height / 4.0;
	double upper_leg_height = wall_height / 8.0, lower_leg_height = wall_height / 8.0;
	double torso_width = width_coefficient * torso_height;
	double upper_leg_width = width_coefficient * upper_leg_height;
	double lower_leg_width = width_coefficient * lower_leg_height;

	GLMmodel m_model ;

} ;
#endif
