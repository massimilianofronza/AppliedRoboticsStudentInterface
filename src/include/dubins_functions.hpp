#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <limits>
//#include "utils.hpp"

namespace student {

	////////// DUBINS FUNCTIONS //////////

	#define PI 3.14159265

	// -------------------------------------------
	// DATATYPES
	// -------------------------------------------

	enum dubins_primitives {
	    LSL, RSR, LSR, RSL, RLR, LRL, MAXIMUM_NUMBER_OF_CURVES
	};

	// New datatype to store the 6 dubins primitives in an array and 
	// lately iterate over them
	// TODO: when the final project is ready, keep in mind that you can use a 
	// switch instead of this to choose function relative to the enum type
	typedef bool (*DubinsFunctionPointer) (
	    double sc_th0, double sc_thf, int sc_Kmax,
	    double& sc_s1, double& sc_s2, double& sc_s3
	);



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

	// Array of pointers to iterate over dubins primitives functions
	DubinsFunctionPointer primitives[6] = {
	    dubins_LSL,
	    dubins_RSR,
	    dubins_LSR,
	    dubins_RSL,
	    dubins_RLR,
	    dubins_LRL
	};

	// Definition of the curvatue signs corresponding to the different
	// dubins primitives functions
	int dubins_primitives_ksigns[6][3] = {
	    {  1,  0,  1 },     // LSL
	    { -1,  0, -1 },     // RSR
	    {  1,  0, -1 },     // LSR
	    { -1,  0,  1 },     // RSL
	    { -1,  1, -1 },     // RLR
	    {  1, -1,  1 }      // LRL
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

	

	// -------------------------------------------
	// DUBINS_CORE
	// -------------------------------------------

	void scaleToStandard(configuration initial, configuration final, 
					double kmax, double &scTh0, double &scThf, 
					double &scKmax, double &lambda);

	void scaleFromStandard(double lambda, double sc_s1, double sc_s2, 
					double sc_s3, double& s1, double& s2, double& s3);

	bool dubins_LSL(double sc_th0, double sc_thf, int sc_Kmax, 
    	            double& sc_s1, double& sc_s2, double& sc_s3);

	bool dubins_RSR(double sc_th0, double sc_thf, int sc_Kmax, 
    	            double& sc_s1, double& sc_s2, double& sc_s3);

	bool dubins_LSR(double sc_th0, double sc_thf, int sc_Kmax, 
    	            double& sc_s1, double& sc_s2, double& sc_s3);

	bool dubins_RSL(double sc_th0, double sc_thf, int sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3);

	bool dubins_RLR(double sc_th0, double sc_thf, int sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3);

	bool dubins_LRL(double sc_th0, double sc_thf, int sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3);

	std::pair<int, dubinsCurve> dubins_shortest_path(configuration initial,
                                                	 configuration final, 
                                                	 int Kmax);


	// -------------------------------------------
	// DUBINS_PLOT
	// -------------------------------------------

	void plot(double **pts_1, cv::Scalar c1, double **pts_2, cv::Scalar c2, 
			  double **pts_3, cv::Scalar c3);

	double** get_arc_points(dubinsArc arc);

	void plot_dubins(dubinsCurve curve, cv::Scalar c1, cv::Scalar c2, 
										cv::Scalar c3);

}
