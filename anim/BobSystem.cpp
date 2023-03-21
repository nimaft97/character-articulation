#include "BobSystem.h"

BobSystem::BobSystem( const std::string& name ):
	BaseSystem( name )
{ 
	
}	// BobSystem

void BobSystem::getState( double* p )
{ 
}	// BobSystem::getState

void BobSystem::setState( double  *p )
{ 
}	// BobSystem::setState

void BobSystem::reset( double time )
{ 	
}	// BobSystem::Reset


int BobSystem::command(int argc, myCONST_SPEC char **argv)
{
	if( argc < 1 )
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str()) ;
		return TCL_ERROR ;
	}
	else if( strcmp(argv[0], "read") == 0 )
	{
		if( argc == 2 )
		{
			m_model.ReadOBJ(argv[1]) ;
			glmFacetNormals(&m_model) ;
			glmVertexNormals(&m_model, 90) ;
			return TCL_OK ;
		}
		else 
		{
			animTcl::OutputMessage("Usage: read <file_name>") ;
			return TCL_ERROR ;
		}
	}
	else if( strcmp(argv[0], "flipNormals") == 0 )
	{
		flipNormals() ;
		return TCL_OK ;
		
	}
	else if( strcmp(argv[0], "reset") == 0)
	{
		double p[3] = {0,0,0} ;
		setState(p) ;
	}
    
    glutPostRedisplay() ;
	return TCL_OK ;

}	// BobSystem::command

void BobSystem::displayClassroom(GLenum)
{
	// draw the classroom

	// floor
	double floor_thickness = 0.8;
	glPushMatrix();
	glTranslated(0, -(torso_height/2.0 + upper_leg_height + lower_leg_height + floor_thickness/2.0 + 0.1), floor_height / 2.0);
	glRotated(90.0, 1.0, 0.0, 0.0);
	glScaled(floor_width, floor_height, floor_thickness);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(1.0, 1.0, 0.0);
	glutSolidCube(1);
	glPopMatrix();

	// blackboard
	glPushMatrix();
	//glTranslated(0.0, blackboard_height, 0.0);
	glScaled(blackboard_width, blackboard_height, 0.8);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 0.0, 0.0);
	glutSolidCube(1);
	glPopMatrix();

	// wall
	glScaled(wall_width, wall_height, 0.5);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(1.0, 1.0, 1.0);
	glutSolidCube(1);
	glPopMatrix();
}

void BobSystem::displayFixedPartsOfBody(GLenum mode)
{
	// draw the character (Bob)
	glPushMatrix();
	//glTranslated(0.0, -torso_height / 2.0, z_distance);
	glTranslated(0.0, 0.0, z_distance);
	// torso
	glPushMatrix();
	glScaled(torso_width, torso_height, 1.0);
	animTcl::OutputMessage("torso height: %f: ", torso_height);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 1.0, 0.0);
	GLdrawCircle(0.5, 500); // so that the diameter is 1
	glPopMatrix();

	// upper legs
	glPushMatrix();
	glTranslated(0.0, -(torso_height + upper_leg_height) / 2.0, 0.0);
	glScaled(upper_leg_width, upper_leg_height, 1);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 1.0, 0.0);
	// right leg
	glPushMatrix();
	glTranslated(torso_width/2.0, 0, 0);
	GLdrawCircle(0.5, 500);
	glPopMatrix();
	// left leg
	glPushMatrix();
	glTranslated(-torso_width/2.0, 0, 0);
	GLdrawCircle(0.5, 500);
	glPopMatrix();
	// upper legs
	glPopMatrix();

	// lower legs
	glPushMatrix();
	glTranslated(0.0, -(torso_height / 2.0 + upper_leg_height + lower_leg_height / 2.0), 0.0);
	glScaled(lower_leg_width, lower_leg_height, 1);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 1.0, 0.0);

	// right leg
	glPushMatrix();
	glTranslated(torso_width/2.0, 0, 0);
	GLdrawCircle(0.5, 500);
	glPopMatrix();

	// left leg
	glPushMatrix();
	glTranslated(-torso_width/2.0, 0, 0);
	GLdrawCircle(0.5, 500);
	glPopMatrix();
	// lower legs
	glPopMatrix();

	// feet
	glPushMatrix();
	glTranslated(0.0, -(torso_height / 2.0 + upper_leg_height + lower_leg_height), -lower_leg_height * (1.0 / 3.0));
	glScaled(lower_leg_height / 4.0, lower_leg_height / 3.0, 1);
	glRotated(95, 1.0, 0.0, 0.0); // rotated more than 90 degrees for better visibility

	// right foot
	glPushMatrix();
	glTranslated(torso_width, 0.0, 0.0);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 1.0, 0.0);
	GLdrawCircle(0.5, 500);
	glPopMatrix();

	// left foot
	glPushMatrix();
	glTranslated(-torso_width, 0.0, 0.0);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 1.0, 0.0);
	GLdrawCircle(0.5, 500);
	glPopMatrix();
	glPopMatrix(); // feet

	// left: arm and hand
	glPushMatrix();
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 1.0, 0.0);
	glTranslated(-torso_width/2.0, 0.0, 0.0);

	//rotation
	glTranslated(-l1_length / 2.0, torso_height / 2.0, 0.0);
	glRotated(-90, 0.0, 0.0, 1.0);

	// shoulder to elbow
	glPushMatrix();
	glScaled(l1_width/2, l1_length, 1);
	GLdrawCircle(0.5, 500);
	glPopMatrix(); // shoulder to elbow

	// elbow to wrist
	glTranslated(0.0, -(l1_length + l2_length) / 2.0, 0.0);
	glPushMatrix();
	glScaled(l2_width / 2.0, l2_length, 1);
	GLdrawCircle(0.5, 500);
	glPopMatrix(); // elbow to wrist

	// wrist to finger
	glTranslated(0.0, -(l2_length + l3_length) / 2.0, 0.0);
	glPushMatrix();
	glScaled(l3_width / 2.0, l3_length, 1);
	GLdrawCircle(0.5, 500);
	glPopMatrix(); // elbow to wrist
	

	glPopMatrix(); // left: arm and hand

	// head
	glPushMatrix();
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 1.0, 0.0);
	glTranslated(0.0, (torso_height + wall_height / 20.0)/2.0, 0.0);
	glScaled(wall_height / 20.0, wall_height / 20.0, 1);
	GLdrawCircle(0.5, 500);
	glPopMatrix(); // head


	glPopMatrix(); // Bob
}

void BobSystem::displayRightHand(GLenum mode)
{
	// draw the character (Bob)
	glPushMatrix();
	glTranslated(0.0, 0.0, z_distance);
	
	// right: arm and hand
	glPushMatrix();
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 1.0, 0.0);

	glTranslated(torso_width/2.0, 0.0, 0.0);

	//rotation
	glTranslated(l1_length/2.0, torso_height/2.0, 0.0);
	glRotated(90, 0.0, 0.0, 1.0);

	// shoulder to elbow
	glPushMatrix();
	glScaled(l1_width / 2, l1_length, 1);
	GLdrawCircle(0.5, 500);
	glPopMatrix(); // shoulder to elbow

	// elbow to wrist
	glTranslated(0.0, -(l1_length + l2_length) / 2.0, 0.0);
	glPushMatrix();
	glScaled(l2_width / 2.0, l2_length, 1);
	GLdrawCircle(0.5, 500);
	glPopMatrix(); // elbow to wrist

	// wrist to finger
	glTranslated(0.0, -(l2_length + l3_length) / 2.0, 0.0);
	glPushMatrix();
	glScaled(l3_width / 2.0, l3_length, 1);
	GLdrawCircle(0.5, 500);
	glPopMatrix(); // elbow to wrist


	glPopMatrix(); // left: arm and hand

	glPopMatrix(); // Bob
}

void BobSystem::display( GLenum mode )
{

	glEnable(GL_LIGHTING) ;
	glMatrixMode(GL_MODELVIEW) ;
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	// push rotation
	glPushMatrix();
	glRotated(45.0, 0.0, 1.0, 0.0);
	// fixed parts of Bob
	displayFixedPartsOfBody(mode);
	// right hand
	displayRightHand(mode);
	// classroom
	displayClassroom(mode);
	// spline
	if (m_spline_is_loaded)
	{
		glPushMatrix();
		glTranslated(0.0, 0.0, 1.0);
		glRotated(45, 0.0, 1.0, 0.0);
		hermiteSystem->display(mode);
		glPopMatrix();
	}
	// pop rotation
	glPopMatrix();


	glPopAttrib();

}	// BobSystem::display
