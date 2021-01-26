#include <iostream>

namespace student {


	// -------------------------------
	// COLLISION_UTILITY
	// -------------------------------

	bool coll_LineLine(double x1, double y1, double x2, double y2,
					   double x3, double y3, double x4, double y4);

	void coll_LineCircle();


	// -------------------------------
	// COLLISION_UTILITY
	// -------------------------------

	double max(double a, double b);

	double min(double a, double b);

	double cross2D(double M_00, double M_01, double M_10, double M_11);

	double dot2D(double M_00, double M_01, double M_10, double M_11);


	// -------------------------------
	// COLLISION_PLOT
	// -------------------------------

	void plotLines(bool res, double x1, double y1, double x2, double y2,
							 double x3, double y3, double x4, double y4);

}