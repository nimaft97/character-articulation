#include <string>
#include <sstream>
#include <fstream>
#include <cctype>
#include <cmath>

#include "HermiteSystem.h"
#include "../util/GLutilities.h"

HermiteSplineSystem::HermiteSplineSystem(const std::string& name) :
	BaseSystem(name),
	m_sx(1.0f),
	m_sy(1.0f),
	m_sz(1.0f)
{

	setVector(m_pos, 0, 0, 0);
	numPoints = 0;

}

void HermiteSplineSystem::getState(double* p)
{

	VecCopy(p, m_pos);

}

void HermiteSplineSystem::setState(double* p)
{

	VecCopy(m_pos, p);

}

void HermiteSplineSystem::reset(double time)
{
	if (!points.empty()) { // the first control point is where the object starts
		setVector(m_pos, points[0][0], points[0][1], points[0][2]);
	}
	else {
		setVector(m_pos, 0.0, 0.0, 0.0);
	}
}


int HermiteSplineSystem::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}

	else if (strcmp(argv[0], "read") == 0) {
		if (argc == 2) {
			points.clear();
			tangents.clear();
			numPoints = 0;
			std::ifstream infile(argv[1]);
			std::string line;
			while (std::getline(infile, line))
			{
				std::istringstream iss(line);

				if (isalpha(line[0])) { // first line contains the name of the file!
					std::string splineName;
					int n;
					iss >> splineName >> n;
					m_name = splineName;
					numPoints = n;
					continue;
				}

				double p1, p2;
				if (!(iss >> p1 >> p2)) { break; } // error
				points.push_back({ p1, p2, 0.0 });
				tangents.push_back({ 0.0, 0.0, 0.0 });
			}
			catmullRomOriginalPoints();
			createTable();
		}
		else {
			animTcl::OutputMessage("Usage: invalid number of arguments");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "export") == 0) {
		if (argc == 2) {
			std::ofstream outfile;
			outfile.open(argv[1]);
			outfile << m_name << " " << numPoints << "\n";
			for (int i = 0; i < numPoints; i++) {
				outfile << points[i][0] << " " << points[i][1] << " " << points[i][2] << " ";
				outfile << tangents[i][0] << " " << tangents[i][1] << " " << tangents[i][2] << "\n";
			}
			outfile.close();
		}
		else {
			animTcl::OutputMessage("Usage: invalid number of arguments");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "set") == 0)
	{
		if (argc == 6)
		{
			int idx = std::stoi(argv[2]);
			double v1 = std::stod(argv[3]), v2 = std::stod(argv[4]), v3 = std::stod(argv[5]);
			if (strcmp(argv[1], "tangent") == 0) {
				tangents[idx] = { v1, v2, v3 };
				return TCL_OK;
			}
			else if (strcmp(argv[1], "point") == 0) {
				points[idx] = { v1, v2, v3 };
				return TCL_OK;
			}
			else {
				animTcl::OutputMessage("Usage: invalid keyword");
				return TCL_ERROR;
			}
		}
		else
		{
			animTcl::OutputMessage("Usage: invalid number of arguments");
			return TCL_ERROR;
		}
		createTable();
	}
	else if (strcmp(argv[0], "add") == 0)
	{
		if (argc == 8)
		{
			double p1 = std::stod(argv[2]), p2 = std::stod(argv[3]), p3 = std::stod(argv[4]);
			double t1 = std::stod(argv[5]), t2 = std::stod(argv[6]), t3 = std::stod(argv[7]);
			if (strcmp(argv[1], "point") == 0) {
				if (numPoints < 40) {
					points.push_back({ p1, p2, p3 });
					tangents.push_back({ t1, t2, t3 });
					numPoints++;
					return TCL_OK;
				}
				else {
					animTcl::OutputMessage("Usage: invalid keyword");
					return TCL_ERROR;
				}

			}
			else {
				animTcl::OutputMessage("Usage: invalid number of arguments");
				return TCL_ERROR;
			}
		}
		createTable();
	}

	glutPostRedisplay();
	return TCL_OK;
}

void HermiteSplineSystem::catmullRomOriginalPoints() {
	/*
	si = (y+1 - yi-1)/2.0 for i = 1,�,n-2
	s0 = 2(y1 - y0) - (y2-y0)/2
	sn-1 = 2(yn-1 - yn-2) - (yn-1 - y n-3)/2
	*/
	for (int i = 1; i < numPoints - 1; i++) {
		for (int j = 0; j < 3; j++) {
			tangents[i][j] = (points[i + 1][j] - points[i - 1][j]) / 2;
		}
	}
	// tangnet of the first point
	for (int j = 0; j < 3; j++) {
		tangents[0][j] = 2 * (points[1][j] - points[0][j]) - (points[2][j] - points[0][j]) / 2;
	}

	// tangnet of the last point
	for (int j = 0; j < 3; j++) {
		tangents[numPoints - 1][j] = 2 * (points[numPoints - 1][j] - points[numPoints - 2][j]) - (points[numPoints - 1][j] - points[numPoints - 3][j]) / 2;
	}
}

void HermiteSplineSystem::catmullRomIntermediatePoint(double x1, double y1, double z1, double x3, double y3, double z3, double& sx, double& sy, double& sz) {
	sx = (x3 - x1) / 2;
	sy = (y3 - y1) / 2;
	sz = (z3 - z1) / 2;
}

double HermiteSplineSystem::calculateTmpVal(double t, double a, double b, double sa, double sb) {
	double answer = 0.0;
	answer += a * (2 * pow(t, 3) - 3 * pow(t, 2) + 1);
	answer += b * (-2 * pow(t, 3) + 3 * pow(t, 2));
	answer += sa * (pow(t, 3) - 2 * pow(t, 2) + t);
	answer += sb * (pow(t, 3) - pow(t, 2));
	return answer;
}

void HermiteSplineSystem::nextPoint(double t, int i, double& next_x, double& next_y, double& next_z) {
	double x1 = points[i][0], y1 = points[i][1], z1 = points[i][2];
	double x2 = points[i + 1][0], y2 = points[i + 1][1], z2 = points[i + 1][2];
	double sx1 = tangents[i][0], sy1 = tangents[i][1], sz1 = tangents[i][2];
	double sx2 = tangents[i + 1][0], sy2 = tangents[i + 1][1], sz2 = tangents[i + 1][2];
	next_x = calculateTmpVal(t, x1, x2, sx1, sx2);
	next_y = calculateTmpVal(t, y1, y2, sy1, sy2);
	next_z = calculateTmpVal(t, z1, z2, sz1, sz2);
}

double HermiteSplineSystem::calcEuclideanDistance(double srcX, double srcY, double srcZ, double dstX, double dstY, double dstZ) {
	return std::sqrt(pow(srcX - dstX, 2) + pow(srcY - dstY, 2) + pow(srcZ - dstZ, 2));
}

void HermiteSplineSystem::createTable() {

	idx2length.clear();
	idx2u.clear();

	double old_x = points[0][0], old_y = points[0][1], old_z = points[0][2];
	double x, y, z;
	int idx = 0;
	double u = 0.0, length = 0.0;
	idx2u.push_back(u);
	idx2length.push_back(length);
	for (int i = 0; i < numPoints - 1; i++) {
		for (double t = 0.0; t <= 1.0; t += deltaT) {
			nextPoint(t, i, x, y, z);
			u += (deltaT / (numPoints - 1));
			idx++;
			length += calcEuclideanDistance(old_x, old_y, old_z, x, y, z);
			idx2u.push_back(u);
			idx2length.push_back(length);
			old_x = x, old_y = y, old_z = z;
		}
	}
}

double HermiteSplineSystem::calcLenghtWithU(double u) {
	createTable();
	int idx = int(u / (deltaT / (numPoints - 1)));
	assert(idx < idx2u.size());
	if (idx == idx2u.size() - 1)	return idx2length[idx];
	double ui = idx2u[idx], ui1 = idx2u[idx + 1];
	return idx2length[idx] + ((u - ui) / (ui1 - ui)) * (idx2length[idx + 1] - idx2length[idx]);
}


void HermiteSplineSystem::display(GLenum mode)
{
	glEnable(GL_COLOR_MATERIAL);


	//glEnable(GL_LIGHTING);
	//glMatrixMode(GL_MODELVIEW);
	//glPushMatrix();
	//glPushAttrib(GL_ALL_ATTRIB_BITS);
	//glTranslated(m_pos[0], m_pos[1], m_pos[2]);
	//glScalef(m_sx, m_sy, m_sz);

	glPushMatrix();

	/*
	// draw control points
	glPointSize(5);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0); // red
	for (int i = 0; i < numPoints; i++) {
		glVertex3d(points[i][0], points[i][1], points[i][2]);
	}
	glEnd();
	glFlush();
	*/

	// draw lines
	// draw lines
	double old_x = points[0][0], old_y = points[0][1], old_z = points[0][2];
	double x, y, z;
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < numPoints - 1; i++) {
		for (double t = 0.0; t <= 1.0; t += deltaT) {
			nextPoint(t, i, x, y, z);
			glColor3f(1.0, 1.0, 1.0);
			//glVertex3d(old_x, old_y, old_z);
			glVertex3d(x, y, z);

			old_x = x, old_y = y, old_z = z;
		}
	}
	glEnd();
	glFlush();



	glPopMatrix();

}
