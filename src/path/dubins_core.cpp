#include "dubins_functions.hpp"

namespace student {

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
        lambda = hypot(dx, dy) / 2.0;

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


	bool dubins_LSL(double sc_th0, double sc_thf, double sc_Kmax, 
                	double& sc_s1, double& sc_s2, double& sc_s3) {
	    double invK = 1.0 / sc_Kmax;
	    double C = cos(sc_thf) - cos(sc_th0);
	    double S = 2.0 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	    double temp1 = atan2(C, S);
	    sc_s1 = invK * mod2pi(temp1 - sc_th0);
	    double temp2 = 2.0 + 4.0 * pow(sc_Kmax, 2) - 2.0 * cos(sc_th0 - sc_thf) 
	                       + 4.0 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
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

	bool dubins_RSR(double sc_th0, double sc_thf, double sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3) {
	    double invK = 1.0 / sc_Kmax;
	    double C = cos(sc_th0) - cos(sc_thf);
	    double S = 2.0 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	    double temp1 = atan2(C, S);
	    sc_s1 = invK * mod2pi(sc_th0 - temp1);
	    double temp2 = 2.0 + 4.0 * pow(sc_Kmax, 2) - 2.0 * cos(sc_th0 - sc_thf)
	                       - 4.0 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
	  
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

	bool dubins_LSR(double sc_th0, double sc_thf, double sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3) {
	    double invK = 1.0 / sc_Kmax;
	    double C = cos(sc_th0) + cos(sc_thf);
	    double S = 2.0 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
	    double temp1 = atan2(-C, S);
	    double temp3 = 4.0 * pow(sc_Kmax, 2) - 2.0 + 2.0 * cos(sc_th0 - sc_thf)
	                 + 4.0 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
	    
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
	bool dubins_RSL(double sc_th0, double sc_thf, double sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3) {

	    double curvature = 1.0 / sc_Kmax;
	    double C = cos(sc_th0) + cos(sc_thf);
	    double S = 2.0 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
	    double temp1 = atan2(C, S);
	    double temp3 = 4.0 * pow(sc_Kmax, 2) - 2.0 + 2.0 * cos(sc_th0 - sc_thf) 
	    			 - 4.0 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));

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
	bool dubins_RLR(double sc_th0, double sc_thf, double sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3) {

	    double curvature = 1 / sc_Kmax;
	    double C = cos(sc_th0) - cos(sc_thf);
	    double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	    double temp1 = atan2(C, S);
	    double temp2 = 0.125 * (6.0 - 4.0 * pow(sc_Kmax, 2) + 2.0 * cos(sc_th0 - sc_thf) 
	    			 + 4.0 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
	    
	    if (abs(temp2) > 1){
	        sc_s1 = 0; 
	        sc_s2 = 0; 
	        sc_s3 = 0;
	        return false;
	    }
	    sc_s2 = curvature * mod2pi(2.0 * PI - acos(temp2));
	    sc_s1 = curvature * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
	    sc_s3 = curvature * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));
	    
	    return true;
	}

	/**
	 * Function to implement the finding of the path made of a Left curve, followed by a  
	 * Right curve, followed by a Left curve. 
	 */
	bool dubins_LRL(double sc_th0, double sc_thf, double sc_Kmax, 
	                double& sc_s1, double& sc_s2, double& sc_s3) {
	    
	    double curvature = 1.0 / sc_Kmax;
	    double C = cos(sc_thf) - cos(sc_th0);
	    double S = 2.0 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	    double temp1 = atan2(C, S);
	    double temp2 = 0.125 * (6.0 - 4.0 * pow(sc_Kmax, 2) + 2.0 * cos(sc_th0 - sc_thf) 
	    			 - 4.0 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));

	    if (abs(temp2) > 1){
	        sc_s1 = 0;
	        sc_s2 = 0; 
	        sc_s3 = 0;
	        return false;
	    }
	    sc_s2 = curvature * mod2pi(2.0 * PI - acos(temp2));
	    sc_s1 = curvature * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
	    sc_s3 = curvature * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));

	    return true;
	}

	// Solve the Dubins problem for the given input parameters.
	// Return the type and the parameters of the optimal curve
	std::pair<int, dubinsCurve> dubins_shortest_path(configuration initial, 
	                                                 configuration final, 
	                                                 double Kmax) {
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
	    pidx = -1;
	    double L = std::numeric_limits<double>::max();
	    double sc_s1 = 0;
	    double sc_s2 = 0;
	    double sc_s3 = 0;

	    // Iterate over the 6 dubins primitives
	    for (int i = LSL; i != MAXIMUM_NUMBER_OF_CURVES; i++) {

	        // Return values of the primitives:
	        bool found = false;
	        double sc_s1_c = 0;
	        double sc_s2_c = 0;
	        double sc_s3_c = 0;

	        switch (i) {
			    case LSL:
					found = dubins_LSL(sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c);
			        break;
			    case RSR:
					found = dubins_RSR(sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c);
			    	break;
			    case LSR:
			        found = dubins_LSR(sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c);
			        break;
			    case RSL:
			        found = dubins_RSL(sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c);
			        break;
			    case RLR:
			        found = dubins_RLR(sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c);
			        break;
			    case LRL:
			        found = dubins_LRL(sc_th0, sc_thf, sc_Kmax, sc_s1_c, sc_s2_c, sc_s3_c);
			        break;
			    default:
			        assert( ! "Invalid Foo enum value" );
			        break;
			}

	        double Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
	        
	        if (found && (Lcur < L)) {
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
	                            dubins_primitives_ksigns[pidx][0] * Kmax, 
	                            dubins_primitives_ksigns[pidx][1] * Kmax, 
	                            dubins_primitives_ksigns[pidx][2] * Kmax);
	        
	        // Check the correctness of the algorithm
	        if (
	        	check (sc_s1, dubins_primitives_ksigns[pidx][0]*sc_Kmax, 
	                   sc_s2, dubins_primitives_ksigns[pidx][1]*sc_Kmax, 
	                   sc_s3, dubins_primitives_ksigns[pidx][2]*sc_Kmax, 
	                   sc_th0, sc_thf
	            )
	        ) {
	        	std::cout << "CHECK IN <dubins_shortest_path> of dubins_core.cpp returned TRUE\n";
	    	}
	    	else {
	        	std::cerr << "________ERROR IN METHOD <dubins_shortest_path> of dubins_core.cpp: algorithm check returned false.________\n";
	        	exit(-1);
	        }
	    }
	    else {
	        std::cerr << "________ERROR IN METHOD <dubins_shortest_path> of dubins_core.cpp: optimal curve not found.________\n";
	        exit(-1);
	    }
	    
	    std::cout << "Best curve: " << pidx << std::endl;
	    return std::pair<int, dubinsCurve>(pidx, curve);
	}

	// Executes for now simple multipoint computations
	void multipoint() {

		double arena_limit[2] = {1.56, 1.06};
		double Kmax = 10;
		int N_POINTS = 3;

		configuration *points;//[N_POINTS];
		points = new (std::nothrow) configuration [N_POINTS];
		if (points == nullptr) {
			std::cerr << "________ERROR IN METHOD <multipoint> of dubins_core.cpp: cannot allocate multi-points.________\n";
			exit(-1);
		}

// can do a loop wherever I will need this
		points[0].x = 0.2;
		points[0].y = 0.2;
		points[0].th = 0;
	
		points[1].x = 0.9;
		points[1].y = 0.8;
		points[1].th = 0;

		points[2].x = 1.4;
		points[2].y = 0.2;
		points[2].th = 0;

		dubinsCurve *curves;
		curves = new dubinsCurve [N_POINTS-1];	// 3 dots == 2 curves
		if (curves == nullptr) {
			std::cerr << "________ERROR IN METHOD <multipoint> of dubins_core.cpp: cannot allocate multi-curves.________\n";
			exit(-1);
		}
/////////////////////////////////
		auto start = startTime();
		
		//for (int i = 1; i < N_POINTS; i++) { // brute force
		for (int i = N_POINTS-1; i > 0; i--) {
			
			std::pair<int, dubinsCurve> tmp;
			tmp = dubins_shortest_path(points[i-1], points[i], Kmax);

			curves[i-1] = tmp.second;
			//plot_dubins(curves[i-1]);
		}

		delete[] points;
		//delete[] curves;	// TODO remember to do this somewhere.
		stopTime(start, false);
/////////////////////////////////
	}
	
}