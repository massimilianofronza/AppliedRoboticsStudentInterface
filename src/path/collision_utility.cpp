#include "collision_functions.hpp"


namespace student {

	double max(double a, double b) {
		return (a>b) ? a : b;
	}

	double min(double a, double b) {
		return (a<b) ? a : b;
	}

	// Computes the cross product of a 2-Dimensional matrix
	// Variables are named after their ficticious index in the matrix
	double cross2D(double M_00, double M_01, double M_10, double M_11) {
		return (M_00 * M_11) - (M_01 * M_10);
	}

	// Computes the dot product of two arrays
	double dot2D(double M_00, double M_01, double M_10, double M_11) {
		return (M_00 * M_10) + (M_01 * M_11);
	}

}
