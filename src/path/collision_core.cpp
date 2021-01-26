#include "collision_functions.hpp"

namespace student {

	// Detects collisions between a couple of segments
	bool coll_LineLine(double x1, double y1, double x2, double y2,
					   double x3, double y3, double x4, double y4) {

		bool res = false;

/*		double x1 = 1;
		double y1 = 1;
		double x2 = 66.57;
		double y2 = 0.367;
		double x3 = 1;
		double y3 = 2;
		double x4 = 56.01;
		double y4 = 10.067;
*/
		// Compute maximum and minimum x and y of the segments
		// TODO compare w.r.t. the <algorithm> max and min
		// First segment:
		double minX1 = min(x1, x2);
		double minY1 = min(y1, y2);
		double maxX1 = max(x1, x2);
		double maxY1 = max(y1, y2);

		// Second segment:
		double minX2 = min(x3, x4);
		double minY2 = min(y3, y4);
		double maxX2 = max(x3, x4);
		double maxY2 = max(y3, y4);

		// Checking boundaries first, trying to avoind the next complex part
		if ( (maxX1 < minX2) || 	// first line totally on the left of the second
			 (maxX2 < minX1) || 	// first line totally on the right of the second
			 (maxY1 < minY2) || 	// first line totally below the second
			 (maxY2 < minY1) ) 		// first line totally above the second
		{
			plotLines(res, x1, y1, x2, y2, x3, y3, x4, y4);
			return res;
		}


		double q[2] = {x1, y1};
		double s[2] = {x2 - x1, y2 - y1};
		
		double p[2] = {x3, y3};
		double r[2] = {x4 - x3, y4 - y3};


		double diffPQ[2] = {q[0] - p[0], q[1] - p[1]};

		double crossRS = cross2D(r[0], r[1], s[0], s[1]);
		double crossDiffR = cross2D(diffPQ[0], diffPQ[1], r[0], r[1]);
		double crossDiffS = cross2D(diffPQ[0], diffPQ[1], s[0], s[1]);


		// Computing the actual collision identification process
		if ((crossRS == 0) && (crossDiffR == 0)) {
			double dotRR = dot2D(r[0], r[1], r[0], r[1]);
			double dotSR = dot2D(s[0], s[1], r[0], r[1]);
			double t0 = dot2D(diffPQ[0], diffPQ[1], r[0], r[1]) / dotRR;
			double t1 = t0 + dotSR / dotRR;

			if (dotSR < 0) {
				if (t0 >= 0 && t1 <= 1) {
					res = true;
				}
			}
			else {
				if (t1 >= 0 && t0 <= 1) {
					res = true;
				}
			}
		}
		else {
			if ((crossRS == 0) && (crossDiffR != 0)) {
				res = false; // TODO: assumed false
			}
			else {
				double t = crossDiffS / crossRS;
				double u = crossDiffR / crossRS;
				
				if ((t >= 0) && (t <= 1) && (u >= 0) && (u <= 1)) {
					res = true;	// *** intersection found ***
				}
			}
		}

		plotLines(res, x1, y1, x2, y2, x3, y3, x4, y4);

		return res;
	}
	
	// Detects collisions between a segment and an arc, identified as a circle
	//void coll_LineCircle(double x1, double y1, double x2, double y2,
	//					 double xC, double yC, double radius) {
	void coll_LineCircle() {

	}

}