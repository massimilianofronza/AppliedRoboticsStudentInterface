#include <cmath>
#include <string>
#include <stdexcept>
#include <sstream>
#include <iostream>

#define PI 3.14159265

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
} ;


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
} ;


// -------------------------------------------
// DECLARATIONS
// -------------------------------------------

configuration getNextConfig(configuration curr, double k, double s);

dubinsArc constructArc(configuration currConf, double k, double L);

dubinsCurve constructDubinsCurve(configuration initialConf, double s1, double s2,
        double s3, double k0, double k1, double k2);

void printConfiguration(configuration config);

void printArc(dubinsArc arc);

double sinc(double x);

double mod2pi(double angle);

double rangeSymm(double angle);

void scaleToStandard(configuration initial, configuration final, double kmax, //this is int?
                    double &scTh0, double &scThf, double &scKmax, double &lambda);

bool check(double s1, double k0, double s2, double k1,
            double s3, double k2, double th0, double thf);
   
bool dubins_RSL(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3);

bool dubins_RLR(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3);

bool dubins_LRL(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3);


int main(){
    exit(0);
}

// -------------------------------------------
// IMPLEMENTATION
// -------------------------------------------

/** 
 * Computes next configuration given the current one, the length of the 
 * arc and the curvature. Applied to get the nextConf in dubinsArc
 * */
configuration getNextConfig(configuration curr, double k, double s){
    configuration next; 
    next.x = curr.x + s * sinc(k * s / 2) * cos(curr.th + k * s / 2);
    next.y = curr.y + s * sinc(k * s / 2) * sin(curr.th + k * s / 2);
    next.th = mod2pi(curr.th + k * s); 
    return next;
}

/**
 * Returns a dubinsArc datastructure, which uniquely identifies the arc
 * given its initial and final configurations, its length and curvature
 * */
dubinsArc constructArc(configuration currConf, double k, double L){
    dubinsArc arc; 
    arc.currentConf = currConf;
    arc.k = k;
    arc.len = L;
    arc.nextConf = getNextConfig(currConf, L, k);
    return arc;
}


/**
 * Constructs the whole Dubin's curve given the initial configuration, 
 * and the precomputed parameters of the three arcs. These are
 * the length of the arcs - s1, s2, s3 - and their curvature - k0, k1, k2.
 * */
dubinsCurve constructDubinsCurve(configuration initialConf, double s1, double s2,
        double s3, double k0, double k1, double k2){

    dubinsCurve curve;
    curve.a1 = constructArc(initialConf, k0, s1);
    curve.a2 = constructArc(curve.a1.nextConf, k1, s2);
    curve.a3 = constructArc(curve.a2.nextConf, k2, s3);
    curve.L = s1 + s2 + s3;
    return curve;
}


// Prints info about the configuration
void printConfiguration(configuration config){
    std::cout << "Point: (x,y) = (" << config.x << "," << config.y << ")" << std::endl;
    std::cout << "Orientation: theta = " <<  config.th << std::endl;
}


// Prints info about arcs that compose the Dubins Curve
void printArc(dubinsArc arc){
    std::cout << "Starting configuration: " << std::endl;
    printConfiguration(arc.currentConf);
    std::cout << "k:" << arc.k << std::endl;
    std::cout << "L:" << arc.len << std::endl;
    std::cout << "Final configuration: " << std::endl;
    printConfiguration(arc.nextConf);
    std::cout << std::endl;
}


// Sinc with Taylor series approximation, used to check correctness of solution 
double sinc(double x){
        double y;
        if (abs(x) < 0.002) {
            //For small values of t use Taylor series approximation
            y = 1 - pow(x,2)/6 * (1 - pow(x,2)/20);
        } else {
            y = sin(x)/x;
        }
        return y;
}


// Normalize an angle, [0, 2*PI) (can be useful for findRobot as well)
double mod2pi(double angle){
    double normalized = angle;
    while (normalized < 0){
        normalized = normalized + 2 * PI;
    }
    while (normalized >= 2 * PI){
     normalized = normalized - 2 * PI;
    }
    return normalized;
}


// Normalize an angular difference (range (-pi, pi])
double rangeSymm(double angle){
    double inRange = angle;
    while (inRange <= -PI){
        inRange = inRange + 2 * PI;
    }
    while ( inRange > PI) {
        inRange = inRange - 2 * PI;
    }
    return inRange;
}


// Scale path finding problem into (-1,0) - (1,0) range 
void scaleToStandard(configuration initial, configuration final, double kmax, //this is int
                    double &scTh0, double &scThf, double &scKmax, double &lambda){
        
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


// Take as input the scaled parameters
bool check(double s1, double k0, double s2, double k1,
            double s3, double k2, double th0, double thf){

    // Negative length 
    if ((s1 < 0 ) && (s2 < 0) && (s3 < 0)){
        std::cout << "Length of found curve is negative. Exiting." << std::endl;
        return false;
    }
    
    int x0 = -1;
    int y0 = 0;
    int xf = 1;
    int yf = 0;

    // Three equations must be satisfied 
    // double or int?  x0 - xf = 2
    double eq1 = x0 - s1 * sinc((1/2) * k0 * s1) * cos(th0 + (1/2) * k0 * s1) 
           + s2 * sinc((1/2) * k1 * s2) * cos(th0 + k0 * s1 + (1/2) * k1 * s2)
           + s3 * sinc((1/2) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1/2) * k2 * s3) - xf;

    // double or int? y0 - yf = 0
    double eq2 = y0 + s1 * sinc((1/2) * k0 * s1) * sin(th0 + (1/2) * k0 * s1) 
           + s2 * sinc((1/2) * k1 * s2) * sin(th0 + k0 * s1 + (1/2) * k1 * s2) 
           + s3 * sinc((1/2) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1/2) * k2 * s3) - yf;

    double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

    return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.0e-10);

}


/**
 * Function to implement the finding of the path made of a Right curve, followed by a   
 * Straight line, followed by a Left curve. 
 */
bool dubins_RSL(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3){

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
                double& sc_s1, double& sc_s2, double& sc_s3){

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
                double& sc_s1, double& sc_s2, double& sc_s3){
    
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


/**
 * OLD CODE with all the variables written in plain, no "configuration" struct.
struct dubinsArc {
    double x0;
    double y0;
    double orientation0; // th0
    double len; // L
    double curv; // k
    double xf;
    double yf;
    double orientationf;
} ;


dubinsArc constructArc(double x0, double y0, double th0,  
            double L, double k){
    dubinsArc arc;
    // start configuration and length of arc
    arc.x0 = x0;
    arc.y0 = y0;
    arc.orientation0 = th0;
    arc.len = L;
    arc.curv = k;
    // end configuration
    circline(L, x0, y0, th0, k, arc.xf, arc.yf, arc.orientationf);
    return arc;


    void circline(double x0, double y0, double th0, double s, double k,
                double& x, double& y, double& theta);

    void scaleToStandard(double x0, double y0, double th0,
                    double xf, double yf, double thf, double kmax, //this is int
                    double &scTh0, double &scThf, double &scKmax, double &lambda);

    // Gets next configuration given initial one, length and curvature of path
    void circline(double x0, double y0, double th0, double s, double k,
            double& x, double& y, double& theta){

        x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2);
        y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2);
        theta = mod2pi(th0 + k * s); 
    */



 