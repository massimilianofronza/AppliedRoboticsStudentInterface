#include <cmath>
#include <string>
#include <vector>
#include <limits>

#define PI 3.14159265


// -------------------------------------------
// DECLARATIONS
// -------------------------------------------

bool dubins_LSL(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3);

bool dubins_RSR(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3);

bool dubins_LSR(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3);

double scaleFromStandard(double lambda, double& sc_s1, double& sc_s2, double& sc_s3);


// -------------------------------------------
// DATATYPES
// -------------------------------------------

// New datatype to store the 6 dubins primitives in an array and 
// lately iterate over them
typedef bool (*DubinsFunctionPointer) (
    double sc_th0, double sc_thf, int sc_Kmax,
    double& sc_s1, double& sc_s2, double& sc_s3
);

enum dubins_primitives {
    LSL, RSR, LSR, RSL, RLR, LRL, MAXIMUM_NUMBER_OF_CURVES
};


// -------------------------------------------
// DATASTRUCTURES 
// -------------------------------------------

// TODO: uncomment the last 3
// Array of pointers to iterate over dubins primitives functions
DubinsFunctionPointer primitives[] = {
    dubins_LSL,
    dubins_RSR,
    dubins_LSR
    //dubins_RSL,
    //dubins_RLR,
    //dubins_LRL
};

// Definition of the curvatue signs corresponding to the different dubins
// primitives functions
int dubins_primitives_ksigns[6][3] = {
    {  1,  0,  1 },     // LSL
    { -1,  0, -1 },     // RSR
    {  1,  0, -1 },     // LSR
    { -1,  0,  1 },     // RSL
    { -1,  1, -1 },     // RLR
    {  1, -1,  1 }      // LRL
};


// Scale the solution to the standard problem back to the original problem
void scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3,
                double& s1, double& s2, double& s3) {
    s1 = sc_s1 * lambda;
    s2 = sc_s2 * lambda;
    s3 = sc_s3 * lambda;
}


// -------------------------------------------
// 6 POTENTIAL OPTIMAL DUBINS CURVES
// -------------------------------------------

bool dubins_LSL(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3) {
    bool ok = false;

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
        return ok;
    }

    sc_s2 = invK * sqrt(temp2);
    sc_s3 = invK * mod2pi(sc_thf - temp1);
    ok = true;

    return ok;
}

bool dubins_RSR(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3) {
    bool ok = false;

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
        return ok;
    }

    sc_s2 = invK * sqrt(temp2);
    sc_s3 = invK * mod2pi(temp1 - sc_thf);
    ok = true;

    return ok;
}


bool dubins_LSR(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3) {
    bool ok = false;

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
        return ok;
    }

    sc_s2 = invK * sqrt(temp3);
    double temp2 = - atan2(-2, sc_s2 * sc_Kmax);
    sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
    sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
    ok = true;

    return ok;
}

int main() {}


// TODO: fit the dubinscurve structure
// TODO: fix the return value when called
// Solve the Dubins problem for the given input parameters.
// Return the type and the parameters of the optimal curve
/*std::pair<int, dubinscurve>*/void dubins_shortest_path(double x0, double y0, double th0, 
                            double xf, double yf, double thf, int Kmax ) {
    // Return values:
    int pidx = 0;
    //dubinscurve curve;

    // Variables for scaleToStandard:
    double sc_th0 = 0;
    double sc_thf = 0;
    double sc_Kmax = 0;
    double lambda = 0;

    // Scale down your curve in the normalized range, so from -1,0 up to 1,0
    //scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax, sc_th0, sc_thf, sc_Kmax, lambda);

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

    if (pidx > 0) {

        // Variables for scaleFromStandard:
        double s1 = 0;
        double s2 = 0;
        double s3 = 0;

        // Transform the solution to the problem from the standard form to the 
        // original problem form (scale the lengths)
        scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3, s1, s2, s3);
    
        // Construct the Dubins curve object with the computed optimal parameters
        /*curve = dubinscurve(x0, y0, th0, s1, s2, s3, 
                            dubins_primitives_ksigns(pidx, 0) * Kmax, 
                            dubins_primitives_ksigns(pidx, 1) * Kmax, 
                            dubins_primitives_ksigns(pidx, 2) * Kmax);
        */
        // Check the correctness of the algorithm
        //assert(check(sc_s1, ksigns(pidx,1)*sc_Kmax, sc_s2, ksigns(pidx,2)*sc_Kmax, sc_s3, ksigns(pidx,3)*sc_Kmax, sc_th0, sc_thf));
    }
    else {
        // TODO lancia qualche eccezione per la curva ottimale non trovata
    }
}
