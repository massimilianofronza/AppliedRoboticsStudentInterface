#include "dubins_functions.hpp"

namespace student {

	// -------------------------------------------
	// AUXILIARY UTILITY FUNCTIONS
	// -------------------------------------------

	// Sinc with Taylor series approximation, used to check correctness of solution 
	double sinc(double x) {
        double y;
        if (abs(x) < 0.002) {
            //For small values of t use Taylor series approximation
            y = 1.0 - pow(x,2) / 6.0 * (1.0 - pow(x,2) / 20.0);
        } else {
            y = sin(x)/x;
        }
        return y;
	}

	// Normalize an angle, [0, 2*PI) (can be useful for findRobot as well)
	double mod2pi(double angle) {
	    double normalized = angle;
	    while (normalized < 0){
	        normalized = normalized + 2.0 * PI;
	    }
	    while (normalized >= 2 * PI){
	     normalized = normalized - 2.0 * PI;
	    }
	    return normalized;
	}

	// Normalize an angular difference (range (-pi, pi])
	double rangeSymm(double angle) {
	    double inRange = angle;
	    while (inRange <= -PI){
	        inRange = inRange + 2.0 * PI;
	    }
	    while ( inRange > PI) {
	        inRange = inRange - 2.0 * PI;
	    }
	    return inRange;
	}

	// Take as input the scaled parameters
	bool check(double s1, double k0, double s2, double k1,
	            double s3, double k2, double th0, double thf) {

	    // Negative length 
	    if ((s1 < 0 ) && (s2 < 0) && (s3 < 0)){
	        std::cout << "Length of found curve is negative. Exiting." << std::endl;
	        return false;
	    }

	    double x0 = -1;
	    double y0 = 0;
	    double xf = 1;
	    double yf = 0;

	    // Three equations must be satisfied
	    double eq1 = x0 + s1 * sinc((1.0/2.0) * k0 * s1) * cos(th0 + (1.0/2.0) * k0 * s1) 
	    				+ s2 * sinc((1.0/2.0) * k1 * s2) * cos(th0 + k0 * s1 + (1.0/2.0) * k1 * s2)
						+ s3 * sinc((1.0/2.0) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1.0/2.0) * k2 * s3) - xf;

	    double eq2 = y0 + s1 * sinc((1.0/2.0) * k0 * s1) * sin(th0 + (1.0/2.0) * k0 * s1) 
	           			+ s2 * sinc((1.0/2.0) * k1 * s2) * sin(th0 + k0 * s1 + (1.0/2.0) * k1 * s2) 
	           			+ s3 * sinc((1.0/2.0) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1.0/2.0) * k2 * s3) - yf;

	    double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);
		
	    double sq = sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3);
	    
	    if (DUBINS_UTILITY_DEBUG) {
			std::cout << "Check():" << std::endl;
			std::cout << "\t eq1: " << eq1 << std::endl;
			std::cout << "\t eq2: " << eq2 << std::endl;
			std::cout << "\t eq3: " << eq3 << std::endl;
			std::cout << "\t sq: " << sq << std::endl;
		}
		
		return (sq < 1.0e-3);
	}

	/** 
	 * Computes next configuration given the current one, the length of the 
	 * arc and the curvature. Applied to get the nextConf in dubinsArc
	 * */
	configuration getNextConfig(configuration curr, double k, double s) {
	    configuration next; 
	    next.x = curr.x + s * sinc(k * s / 2.0) * cos(curr.th + k * s / 2.0);
	    next.y = curr.y + s * sinc(k * s / 2.0) * sin(curr.th + k * s / 2.0);
	    next.th = mod2pi(curr.th + k * s); 
	    return next;
	}

	/**
	 * Returns a dubinsArc datastructure, which uniquely identifies the arc
	 * given its initial and final configurations, its length and curvature
	 * */
	dubinsArc constructArc(configuration currConf, double k, double L) {
	    dubinsArc arc; 
	    arc.currentConf = currConf;
	    arc.k = k;
	    arc.len = L;
	    arc.nextConf = getNextConfig(currConf, k, L);
	    return arc;
	}

	/**
	 * Constructs the whole Dubin's curve given the initial configuration, 
	 * and the precomputed parameters of the three arcs. These are
	 * the length of the arcs - s1, s2, s3 - and their curvature - k0, k1, k2.
	 * */
	dubinsCurve constructDubinsCurve(configuration initialConf, double s1, 
					double s2, double s3, double k0, double k1, double k2) {

	    dubinsCurve curve;
	    curve.a1 = constructArc(initialConf, k0, s1);
	    curve.a2 = constructArc(curve.a1.nextConf, k1, s2);
	    curve.a3 = constructArc(curve.a2.nextConf, k2, s3);

	    if (DUBINS_UTILITY_DEBUG) {
	    	printArc(curve.a1);
	    	printArc(curve.a2);
	    	printArc(curve.a3);
	    }

	    curve.L = s1 + s2 + s3;
	    return curve;
	}

	// Prints info about the configuration
	void printConfiguration(configuration config) {
	    std::cout << "Point: (x,y) = (" << config.x << "," << config.y << ")" << std::endl;
	    std::cout << "Orientation: theta = " <<  config.th << std::endl;
	}

	// Prints info about arcs that compose the Dubins Curve
	void printArc(dubinsArc arc) {
	    std::cout << "Starting configuration: " << std::endl;
	    printConfiguration(arc.currentConf);
	    std::cout << "k:" << arc.k << std::endl;
	    std::cout << "L:" << arc.len << std::endl;
	    std::cout << "Final configuration: " << std::endl;
	    printConfiguration(arc.nextConf);
	    std::cout << std::endl;
	}

	// Returns this moment in time
	std::chrono::high_resolution_clock::time_point startTime() {
		return std::chrono::high_resolution_clock::now();
	}

	// Gets the start moment and tells how much time passed between that and now
	void stopTime(std::chrono::high_resolution_clock::time_point start, bool unit) {
		auto stop = std::chrono::high_resolution_clock::now();

		if (unit) {
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
			std::cout << "PERFORMANCE:\n\t" << duration.count() << " milliseconds.\n";
		}
		else {
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
			std::cout << "PERFORMANCE:\n\t" << duration.count() << " microseconds.\n";
		}
	}

}