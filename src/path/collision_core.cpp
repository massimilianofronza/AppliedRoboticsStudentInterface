#include "collision_functions.hpp"

namespace student {

	/**
	 * Detects collisions between a couple of segments, and returns the boolean outcome.
	 */
	bool coll_LineLine(double x1, double y1, double x2, double y2,
					   double x3, double y3, double x4, double y4) {

		bool res = false;

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
			//plotXLine(res, x1, y1, x2, y2, x3, y3, x4, y4);
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

		if (res) {
			//plotXLine(res, x1, y1, x2, y2, x3, y3, x4, y4);
		}
		return res;
	}
	
	/**
	 * Detects collisions between a segment and an arc, identified as a circle
	 */
	bool coll_LineCircle(double x1, double y1, double x2, double y2, dubinsArc arc) {

		bool res = false;
		
		/// Ensure that an arc was passed, not a straight line.
		/// Stop the execution to avoid problems.
		if (arc.k == 0) {
			std::cerr << "________ERROR IN METHOD <coll_LineCircle> of collision_core.cpp: a straight line was passed instead of an arc.________\n";
			exit(-1);
		}
		
		double radius = 1.0 / arc.k;
		if (radius < 0) {		/// Make positive if not
			radius -= radius*2.0;
		}

		double xC = arc.currentConf.x - sin(arc.currentConf.th) * radius;
		double yC = arc.currentConf.y + cos(arc.currentConf.th) * radius;

		if (DEBUG_COLL) {
			std::cout << "RADIUS: " << radius << "\n";
			std::cout << "CENTER: " << xC << ", " << yC << "\n";
		}		

		double p1 = 2.0 * x1 * x2;
		double p2 = 2.0 * y1 * y2;
		double p3 = 2.0 * xC * x1;
		double p4 = 2.0 * xC * x2;
		double p5 = 2.0 * yC * y1;
		double p6 = 2.0 * yC * y2;

		double a = pow(x1, 2.0) + pow(x2, 2.0) - p1 + pow(y1, 2.0) + pow(y2, 2.0) - p2;
		double b = -2.0 * pow(x2, 2.0) + p1 - p3 + p4 -2.0 * pow(y2, 2.0) + p2 - p5 + p6;
		double c = pow(x2, 2.0) - p4 + pow(xC, 2.0) + pow(y2, 2.0) - p6 + pow(yC, 2.0) - pow(radius, 2.0);


		double delta = pow(b, 2.0) - 4.0*a*c;

		if (DEBUG_COLL) {
			std::cout << "DELTA: " << delta << std::endl;
		}

		double t1 = -1;			/// To avoid being considered as collisions
		double t2 = -2;

		if (delta < 0) {		/// No intersection points found
			res = false;
		}
		else {
			if (delta > 0) {	/// 2 intersection points with circle
				double deltaSq = sqrt(delta);
				t1 = (-b + deltaSq) / (2*a);
				t2 = (-b - deltaSq) / (2*a);
			}
			else {				/// 1 collision point with circle, delta equal to zero
				t1 = -b / (2*a);
				t2 = t1;
			}
		}

		/// Find if t1 and t2 are a collision of the actual segment or just
		/// of the rect containing it
		double xT[2] = {-1, -1};
		double yT[2] = {-1, -1};
		int collisions = 0;

		if ((t1 >= 0) && (t1 <= 1)) {
		    xT[collisions] = x1 * t1 + x2 * (1-t1);
		    yT[collisions] = y1 * t1 + y2 * (1-t1);
		    collisions++;
		    res = true;
		}

		if ((t2 >= 0) && (t2 <= 1) && (t2 != t1)) {
			xT[collisions] = x1 * t2 + x2 * (1-t2);
			yT[collisions] = y1 * t2 + y2 * (1-t2);
			collisions++;
			res = true;
		}

		/// Defining angles of collisions w.r.t. the circle center
		double th_T[2] = {0, 0};
		
		if (collisions > 0) {
			for (int i=0; i<collisions; i++) {
				th_T[i] = atan2(yT[i] - yC, xT[i] - xC);
			}
		}
		else {	/// No actual collision
			if (DEBUG_COLL) {
				plotXCircle(res, x1, y1, x2, y2, xC, yC, radius, 
					cv::Point(arc.currentConf.x, arc.currentConf.y),
					cv::Point(arc.nextConf.x, arc.nextConf.y), arc.k);
			}
			return res;		/// Would be false
		}

		if (DEBUG_COLL) {
			std::cout << "COLLISION WITH CIRCLE FOUND.\n";
		}

		/// Defining angles of the two points defining the arc w.r.t. the circle center
		double th_Arc[2] = {0, 0};
		th_Arc[0] = atan2(arc.currentConf.y - yC, arc.currentConf.x - xC);
		th_Arc[1] = atan2(arc.nextConf.y - yC,    arc.nextConf.x - xC);

		if (DEBUG_COLL) {
			std::cout << "Collisions: " << collisions << ", [" << xT[0] << ", " << yT[0] << "] & [" << xT[1] << ", " << yT[1] << "]\n";
			std::cout << "Coll angles: [" << th_T[0] << ", " << th_T[1] << "]\n";
			std::cout << "Arc angles: [" << th_Arc[0] << ", " << th_Arc[1] << "]\n";
		}


		/// Arc peforming a left turn, make each angle positive.
		if (arc.k > 0) {
			if (th_Arc[0] < 0) {
				th_Arc[0] += 2.0*PI;
			}
			if (th_Arc[1] < 0) {
				th_Arc[1] += 2.0*PI;
			}
			
			/// If there was no collision, the function would have already returned.
			if (th_T[0] < 0) {
				th_T[0] += 2.0*PI;
			}
			if (collisions == 2) {
				if (th_T[1] < 0) {
					th_T[1] += 2.0*PI;
				}
			}
		}
		/// If the arc peforms a right turn, make each angle negative.
		else if (arc.k < 0) {
			if (th_Arc[0] > 0) {
				th_Arc[0] -= 2.0*PI;
			}
			if (th_Arc[1] > 0) {
				th_Arc[1] -= 2.0*PI;
			}
			
			/// If there was no collision, the function would have already returned.
			if (th_T[0] > 0) {
				th_T[0] -= 2.0*PI;
			}
			if (collisions == 2) {
				if (th_T[1] > 0) {
					th_T[1] -= 2.0*PI;
				}
			}
		}

		if (DEBUG_COLL) {
			std::cout << "Coll angles after processing: [" << th_T[0] << ", " << th_T[1] << "]\n";
			std::cout << "Arc angles after processing: [" << th_Arc[0] << ", " << th_Arc[1] << "]\n";
			std::cout << "NOW VERIFYING THE CIRCLE COLLISION.\n";
		}
		
		/// Final comparison between angles
		int i = 0;
		while (i < collisions) {

			if (arc.k > 0) {				/// Left turn
				if (th_Arc[1] >= th_Arc[0]) {
					if ((th_Arc[1] >= th_T[i]) && (th_T[i] >= th_Arc[0])) {	/// Real arc collision
						res = true;
						i = collisions;		/// Exit, collision found
					}
					else {
						res = false;
					}
				}
				else {
					if ((th_Arc[1] < th_T[i]) && (th_T[i] < th_Arc[0])) {
						res = false;
					}
					else {													/// Real arc collision
						res = true;
						i = collisions;		/// Exit, collision found						
					}
				}
			}

			else if (arc.k < 0) {			/// Right turn
				if (th_Arc[1] <= th_Arc[0]) {
					if ((th_Arc[1] <= th_T[i]) && (th_T[i] <= th_Arc[0])) {	/// Real arc collision
						res = true;
						i = collisions;		/// Exit, collision found
					}
					else {
						res = false;
					}
				}
				else {
					if ((th_Arc[1] > th_T[i]) && (th_T[i] > th_Arc[0])) {
						res = false;
					}
					else {													/// Real arc collision
						res = true;
						i = collisions;		/// Exit, collision found
					}
				}
			}

			i++;
		}

		if (DEBUG_COLL) {
			std::cout << "FINAL COLLISION RESULT: " << res << ".\n";
			plotXCircle(res, x1, y1, x2, y2, xC, yC, radius, 
				cv::Point(arc.currentConf.x, arc.currentConf.y),
				cv::Point(arc.nextConf.x, arc.nextConf.y), arc.k);
		}
		return res;
	}

}
