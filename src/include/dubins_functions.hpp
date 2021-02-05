#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <limits>
#include <chrono>	// for performance testing
#include "utils.hpp"

namespace student {

	////////// DUBINS FUNCTIONS //////////

	#define PI 3.14159265
	#define DUBINS_UTILITY_DEBUG false
	#define PLOT_X_SIZE 700
	#define PLOT_Y_SIZE 500



	// -------------------------------------------
	// DATATYPES
	// -------------------------------------------

	enum dubins_primitives {
	    LSL, RSR, LSR, RSL, RLR, LRL, MAXIMUM_NUMBER_OF_CURVES
	};



	// -------------------------------------------
	// DATASTRUCTURES 
	// -------------------------------------------

	// A simple datastructure for the pose of the robot
	// implementation choice: this is done to bind together the information, 
	// and work on configurations rather than passing every time x0, y0, th0 and so on 
	// (less parameters needed, more control, more clear)
	struct configuration {
	    double x;
	    double y;
	    double th;
	};

	// An arc of the Dubin's curve, identified by an initial and final 
	// configuration, and by its length and curvature
	struct dubinsArc {
	    configuration currentConf;
	    double len;
	    double k;
	    configuration nextConf;
	};

	// Dubins curve composed by 3 arcs, with total length already computed
	struct dubinsCurve {
	    dubinsArc a1;
	    dubinsArc a2;
	    dubinsArc a3;
	    double L;
	};



	// -------------------------------------------
	// DUBINS_UTILITY
	// -------------------------------------------

	double sinc(double x);

	double mod2pi(double angle);

	double rangeSymm(double angle);

	bool check(double s1, double k0, double s2, double k1,
	           double s3, double k2, double th0, double thf);

	configuration getNextConfig(configuration curr, double k, double s);

	dubinsArc constructArc(configuration currConf, double k, double L);

	dubinsCurve constructDubinsCurve(configuration initialConf, double s1,
			 		double s2, double s3, double k0, double k1, double k2);

	void printConfiguration(configuration config);

	void printArc(dubinsArc arc);

	std::chrono::high_resolution_clock::time_point startTime();

	void stopTime(std::chrono::high_resolution_clock::time_point start, bool unit);

	

	// -------------------------------------------
	// DUBINS_CORE
	// -------------------------------------------

	void scaleToStandard(configuration initial, configuration final, 
					double kmax, double &scTh0, double &scThf, 
					double &scKmax, double &lambda);

	void scaleFromStandard(double lambda, double sc_s1, double sc_s2, 
					double sc_s3, double& s1, double& s2, double& s3);

	bool dubins_LSL(double sc_th0, double sc_thf, double sc_Kmax, 
    	            double& sc_s1, double& sc_s2, double& sc_s3);

	bool dubins_RSR(double sc_th0, double sc_thf, double sc_Kmax, 
    	            double& sc_s1, double& sc_s2, double& sc_s3);

	bool dubins_LSR(double sc_th0, double sc_thf, double sc_Kmax, 
    	            double& sc_s1, double& sc_s2, double& sc_s3);

	bool dubins_RSL(double sc_th0, double sc_thf, double sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3);

	bool dubins_RLR(double sc_th0, double sc_thf, double sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3);

	bool dubins_LRL(double sc_th0, double sc_thf, double sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3);

	std::pair<int, dubinsCurve> dubins_shortest_path(configuration initial,
                                                	 configuration final, 
                                                	 double Kmax);

	std::vector<dubinsCurve> multipoint(const configuration& robot, std::vector<Point>& points);


	// -------------------------------------------
	// DUBINS_PLOT
	// -------------------------------------------

	void plot(cv::Mat image, double **pts_1, cv::Scalar c1, 
							 double **pts_2, cv::Scalar c2, 
			  				 double **pts_3, cv::Scalar c3);

	double** get_arc_points(dubinsArc arc, int npts);

	
    void plot_dubins(std::vector<dubinsCurve> curves);
}
