#include <opencv2/opencv.hpp>
//#include <iostream>	already in dubins_functions TODO leva tutto
//#include <cmath>


namespace student {

	#define PI 3.14159265358979323846
	#define DEBUG_COLL true
	#define COLL_PLOT_X_SIZE 700
	#define COLL_PLOT_Y_SIZE 500


	/*********************
	 * COLLISION_CORE
	 ********************/

	bool coll_LineLine(double x1, double y1, double x2, double y2,
					   double x3, double y3, double x4, double y4);

	bool coll_LineCircle(double x1, double y1, double x2, double y2, 
							dubinsArc arc);


	/*********************
	 * COLLISION_UTILITY
	 ********************/

	double max(double a, double b);

	double min(double a, double b);

	double cross2D(double M_00, double M_01, double M_10, double M_11);

	double dot2D(double M_00, double M_01, double M_10, double M_11);


	/*********************
	 * COLLISION_PLOT
	 ********************/

	void plotXLine(bool res, double x1, double y1, double x2, double y2,
							  double x3, double y3, double x4, double y4);

	void plotXCircle(bool res, double x1, double y1, double x2, double y2,
							   double xC, double yC, double radius,
							   cv::Point a1, cv::Point a2, double verse);

}