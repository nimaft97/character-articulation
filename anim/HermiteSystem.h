#ifndef MY_HERMITE_SPLINE_SYSTEM_H
#define MY_HERMITE_SPLINE_SYSTEM_H

/*

	This is a sample system. It accepts the command "read" followed by the
	path to an OBJ model.

*/


#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include <vector>
#include <unordered_map>

#include "shared/opengl.h"

class HermiteSplineSystem : public BaseSystem
{

public:
	HermiteSplineSystem(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	void display(GLenum mode = GL_RENDER);

	void readModel(char* fname) { m_model.ReadOBJ(fname); }
	void flipNormals(void) { glmReverseWinding(&m_model); }
	int command(int argc, myCONST_SPEC char** argv);
	double calculateTmpVal(double t, double a, double b, double sa, double sb);
	void nextPoint(double t, int i, double& tmp_x, double& tmp_y, double& tmp_z);
	double calcEuclideanDistance(double srcX, double srcY, double srcZ, double dstX, double dstY, double dstZ);
	double calcLenghtWithU(double u);
	void createTable();
	void catmullRomOriginalPoints();
	void catmullRomIntermediatePoint(double x1, double y1, double z1, double x3, double y3, double z3, double& sx, double& sy, double& sz);

public:
	int numPoints;
	double deltaT = 0.001;
	std::vector<std::vector<double>> points, tangents;
	std::vector<double> idx2u, idx2length;

protected:

	float m_sx;
	float m_sy;
	float m_sz;

	Vector m_pos;

	GLMmodel m_model;

};
#endif
