#include <opencv2/core/core.hpp>
#include <iostream>
#include <cmath>
#include <string>
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

std::pair<int, dubinsCurve> dubins_shortest_path(configuration initial,
                                                 configuration final, 
                                                 int Kmax);


// -------------------------------------------
// DATATYPES
// -------------------------------------------

// New datatype to store the 6 dubins primitives in an array and 
// lately iterate over them
// TODO: when the final project is ready, keep in mind that you can use a 
// switch instead of this to choose function relative to the enum type
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
DubinsFunctionPointer primitives[6] = {
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


// Solve the Dubins problem for the given input parameters.
// Return the type and the parameters of the optimal curve
std::pair<int, dubinsCurve> dubins_shortest_path(configuration initial, 
                                                 configuration final, int Kmax ) {
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

    // TODO i changed to >= from > for matlab conversion
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
        
        // Check the correctness of the algorithm - TODO i changed indexes for matlab conversion
        bool check_alg = check(sc_s1, dubins_primitives_ksigns[pidx][0]*sc_Kmax, 
                               sc_s2, dubins_primitives_ksigns[pidx][1]*sc_Kmax, 
                               sc_s3, dubins_primitives_ksigns[pidx][2]*sc_Kmax, 
                               sc_th0, sc_thf);

        if (check_alg != true) {
            std::cerr << "ERROR IN METHOD <dubins_shortest_path> of TODO.cpp: algorithm check returned false." << std::endl;
        }
    }
    else {
        std::cerr << "ERROR IN METHOD <dubins_shortest_path> of TODO.cpp: optimal curve not found." << std::endl;
    }

    // TODO check syntax
    return std::pair<int, dubinsCurve>(pidx, curve);
}


// -------------------------------------------
// PLOT FUNCTIONS
// -------------------------------------------

// Plot of the arc's points on an empty image
void plot(double **pts_1, cv::Scalar c1, 
            double **pts_2, cv::Scalar c2, 
            double **pts_3, cv::Scalar c3) {

    // The resulting image
    cv::Mat image = cv::Mat::zeros(600, 1000, CV_8UC3);
    
    // Setting up a white background
    image.setTo(cv::Scalar(255, 255, 255));
    
    // Linking all consecutive points with a line
    for (int r=0; r<99; r++) {
        cv::line(image, cv::Point(pts_1[r][0], pts_1[r][1]), 
                        cv::Point(pts_1[r+1][0], pts_1[r+1][1]), c1, 4, cv::LINE_8);
        
        cv::line(image, cv::Point(pts_2[r][0], pts_2[r][1]), 
                        cv::Point(pts_2[r+1][0], pts_2[r+1][1]), c2, 4, cv::LINE_8);
        
        cv::line(image, cv::Point(pts_3[r][0], pts_3[r][1]), 
                        cv::Point(pts_3[r+1][0], pts_3[r+1][1]), c3, 4, cv::LINE_8);
    }

    // Free the memory
    delete(pts_1);
    delete(pts_2);
    delete(pts_3);
    
    // Plot the final image
    cv::namedWindow("Dubins curve", cv::WINDOW_AUTOSIZE);
    cv::imshow("Dubins curve", image);
    cv::waitKey(0);
}

// Evaluate an arc (circular or straight) at a given arc-length s
void circline(double s, configuration init, configuration& temp, int k) {
    temp.x = init.x + s * sinc(k * s / 2.0) * cos(init.th + k * s / 2);
    temp.y = init.y + s * sinc(k * s / 2.0) * sin(init.th + k * s / 2);
    temp.th = mod2pi(init.th + k * s);
}

// Plot an arc (circular or straight)
double** get_arc_points(dubinsArc arc) {
    int npts = 100;
    configuration temp;
    
    // Creating the points matrix, with 2 standing for x and y coordinates
    double **pts;
    pts = new double *[npts];
    for(int i=0; i<npts; i++) {
        pts[i] = new double[2];
    }

    // Obtaining points
    for (int j=0; j<npts; j++) {
        double s = arc.len / npts * j;
        circline(s, arc.currentConf, temp, arc.k);
        pts[j][0] = temp.x;
        pts[j][1] = temp.y;
    }
    
    return pts;
}

// Plot a Dubins curve
void plot_dubins(dubinsCurve curve, cv::Scalar c1, cv::Scalar c2, cv::Scalar c3) {
    // Three set of points, one for each arc
    double **pts_1;
    double **pts_2;
    double **pts_3;

    // Get points the three arcs
    pts_1 = get_arc_points(curve.a1);
    pts_2 = get_arc_points(curve.a2);
    pts_3 = get_arc_points(curve.a3);

    plot(pts_1, c1, pts_2, c2, pts_3, c3);
}
