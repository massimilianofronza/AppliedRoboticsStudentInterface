#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "dubins_functions.hpp"

namespace student {

	// -------------------------------------------
	// METHODS TO SCALE AND SOLVE DUBINS PROBLEMS
	// -------------------------------------------

	// Scale path finding problem into (-1,0) - (1,0) range 
	void scaleToStandard(configuration initial, configuration final, 
					double kmax, double &scTh0, double &scThf, 
					double &scKmax, double &lambda) {
        
        // transform parameters (displacement, rotation, scaling factor)
        double dx = final.x - initial.x; // xf - x0 
        double dy = final.y - initial.y; // yf - y0
        double phi = std::atan2(dy, dx);
        lambda = hypot(dx, dy) / 2;

        // apply scaling and normalization (to orientation and curvature)
        scTh0 = mod2pi(initial.th - phi); // th0 - phi 
        scThf = mod2pi(final.th - phi); // thf - phi
        scKmax = kmax * lambda;
	}

	// Scale the solution to the standard problem back to the original problem
	void scaleFromStandard(double lambda, double sc_s1, double sc_s2, 
					double sc_s3, double& s1, double& s2, double& s3) {
	    s1 = sc_s1 * lambda;
	    s2 = sc_s2 * lambda;
	    s3 = sc_s3 * lambda;
	}


	bool dubins_LSL(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3) {
	    double invK = 1 / sc_Kmax;
	    double C = cos(sc_thf) - cos(sc_th0);
	    double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	    double temp1 = atan2(C, S);
	    sc_s1 = invK * mod2pi(temp1 - sc_th0);
	    double temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf) 
	                     + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));

	    if (temp2 < 0) {
	        sc_s1 = 0;
	        sc_s2 = 0;
	        sc_s3 = 0;
	        return false;
	    }

	    sc_s2 = invK * sqrt(temp2);
	    sc_s3 = invK * mod2pi(sc_thf - temp1);
	    return true;
	}

	bool dubins_RSR(double sc_th0, double sc_thf, int sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3) {
	    double invK = 1 / sc_Kmax;
	    double C = cos(sc_th0) - cos(sc_thf);
	    double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	    double temp1 = atan2(C, S);
	    sc_s1 = invK * mod2pi(sc_th0 - temp1);
	    double temp2 = 2 + 4 * pow(sc_Kmax, 2) - 2 * cos(sc_th0 - sc_thf)
	                     - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
	  
	    if (temp2 < 0) {
	        sc_s1 = 0;
	        sc_s2 = 0;
	        sc_s3 = 0;
	        return false;
	    }

	    sc_s2 = invK * sqrt(temp2);
	    sc_s3 = invK * mod2pi(temp1 - sc_thf);
	    return true;
	}

	bool dubins_LSR(double sc_th0, double sc_thf, int sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3) {
	    double invK = 1 / sc_Kmax;
	    double C = cos(sc_th0) + cos(sc_thf);
	    double S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
	    double temp1 = atan2(-C, S);
	    double temp3 = 4 * pow(sc_Kmax, 2) - 2 + 2 * cos(sc_th0 - sc_thf)
	                 + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
	    
	    if (temp3 < 0) {
	        sc_s1 = 0;
	        sc_s2 = 0;
	        sc_s3 = 0;
	        return false;
	    }

	    sc_s2 = invK * sqrt(temp3);
	    double temp2 = - atan2(-2, sc_s2 * sc_Kmax);
	    sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
	    sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
	    return true;
	}

	/**
	 * Function to implement the finding of the path made of a Right curve, followed by a   
	 * Straight line, followed by a Left curve. 
	 */
	bool dubins_RSL(double sc_th0, double sc_thf, int sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3) {

	    double curvature = 1/sc_Kmax;
	    double C = cos(sc_th0) + cos(sc_thf);
	    double S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
	    double temp1 = atan2(C, S);
	    double temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
	    if (temp3 < 0){
	        sc_s1 = 0; 
	        sc_s2 = 0; 
	        sc_s3 = 0;
	        return false;
	    }
	    
	    sc_s2 = curvature * sqrt(temp3);
	    double temp2 = atan2(2, sc_s2 * sc_Kmax);
	    sc_s1 = curvature * mod2pi(sc_th0 - temp1 + temp2);
	    sc_s3 = curvature * mod2pi(sc_thf - temp1 + temp2);

	    return true;
	}


	/** 
	 * Function to implement the finding of the path made of a Right curve, followed by a 
	 * Left curve, followed by a Right curve. 
	 */
	bool dubins_RLR(double sc_th0, double sc_thf, int sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3) {

	    double curvature = 1 / sc_Kmax;
	    double C = cos(sc_th0) - cos(sc_thf);
	    double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	    double temp1 = atan2(C, S);
	    double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
	    
	    if (abs(temp2) > 1){
	        sc_s1 = 0; 
	        sc_s2 = 0; 
	        sc_s3 = 0;
	        return false;
	    }
	    sc_s2 = curvature * mod2pi(2 * PI - acos(temp2));
	    sc_s1 = curvature * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
	    sc_s3 = curvature * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));
	    
	    return true;
	}

	/**
	 * Function to implement the finding of the path made of a Left curve, followed by a  
	 * Right curve, followed by a Left curve. 
	 */
	bool dubins_LRL(double sc_th0, double sc_thf, int sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3) {
	    
	    double curvature = 1 / sc_Kmax;
	    double C = cos(sc_thf) - cos(sc_th0);
	    double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	    double temp1 = atan2(C, S);
	    double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
	    if (abs(temp2) > 1){
	        sc_s1 = 0;
	        sc_s2 = 0; 
	        sc_s3 = 0;
	        return false;
	    }
	    sc_s2 = curvature * mod2pi(2 * PI - acos(temp2));
	    sc_s1 = curvature * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
	    sc_s3 = curvature * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));

	    return true;
	}

	// Solve the Dubins problem for the given input parameters.
	// Return the type and the parameters of the optimal curve
	std::pair<int, dubinsCurve> dubins_shortest_path(configuration initial, 
	                                                 configuration final, 
	                                                 int Kmax ) {
	    // Return values:
	    int pidx;
	    dubinsCurve curve;

	    // Variables for scaleToStandard:
	    double sc_th0 = 0;
	    double sc_thf = 0;
	    double sc_Kmax = 0;
	    double lambda = 0;

	    // Scale down your curve in the normalized range, so from -1,0 up to 1,0
	    scaleToStandard(initial, final, Kmax, sc_th0, sc_thf, sc_Kmax, lambda);

	    // Variables to solve the sys of equations for all of the dubins_primitives
	    // to find the optimal solution:
	    pidx = 0;
	    double L = std::numeric_limits<double>::max();
	    double sc_s1 = 0;
	    double sc_s2 = 0;
	    double sc_s3 = 0;

	    // Iterate over the 6 dubins primitives
	    for (int i = LSL; i != MAXIMUM_NUMBER_OF_CURVES; i++) {

	        // Return values of the primitives:
	        bool ok = false;
	        double sc_s1_c = 0;
	        double sc_s2_c = 0;
	        double sc_s3_c = 0;

	        ok = primitives[i](sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c);
	        double Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
	        
	        if (ok && (Lcur < L)) {
	            L = Lcur;
	            sc_s1 = sc_s1_c;
	            sc_s2 = sc_s2_c;
	            sc_s3 = sc_s3_c;
	            pidx = i;
	        }
	    }

	    if (pidx >= 0) {
	        // Variables for scaleFromStandard:
	        double s1 = 0;
	        double s2 = 0;
	        double s3 = 0;

	        // Transform the solution to the problem from the standard form to the 
	        // original problem form (scale the lengths)
	        scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);
	    
	        // Construct the Dubins curve object with the computed optimal parameters
	        curve = constructDubinsCurve(initial, s1, s2, s3, 
	                            dubins_primitives_ksigns(pidx, 0) * Kmax, 
	                            dubins_primitives_ksigns(pidx, 1) * Kmax, 
	                            dubins_primitives_ksigns(pidx, 2) * Kmax);
	        
	        // Check the correctness of the algorithm
	        bool check_alg = check(sc_s1, dubins_primitives_ksigns[pidx][0]*sc_Kmax, 
	                               sc_s2, dubins_primitives_ksigns[pidx][1]*sc_Kmax, 
	                               sc_s3, dubins_primitives_ksigns[pidx][2]*sc_Kmax, 
	                               sc_th0, sc_thf);

	        if (check_alg != true) {
	            std::cerr << "ERROR IN METHOD <dubins_shortest_path> of dubins_core.cpp: algorithm check returned false." << std::endl;
	        }
	    }
	    else {
	        std::cerr << "ERROR IN METHOD <dubins_shortest_path> of dubins_core.cpp: optimal curve not found." << std::endl;
	    }

	    return std::pair<int, dubinsCurve>(pidx, curve);
	}
	
}