#include <cmath>
#include <string>

#define PI 3.14159265
// -------------------------------------------
// DATASTRUCTURES 
// -------------------------------------------

// L, S, R
struct dubins_arc {

}

// 3 arcs, the whole curve
struct dubinscurve {

}


// -------------------------------------------
// DECLARATIONS
// -------------------------------------------

double sinc(double x);
double mod2pi(double angle);
double rangeSymm(double angle);
void circline(double s, double x0, double y0, double th0, double k,
                double& x, double& y, double& theta);

void scaleToStandard(double x0, double y0, double th0,
                    double xf, double yf, double thf, double kmax, //this is int
                    double &scTh0, double &scThf, double &scKmax, double &lambda);

void scaleFromStandard(double lambda, double scS1, double scS2, double scS3,
                       double &s1, double &s2, double &s3);





// Sinc with Taylor series approximation, used to check correctness of solution 
double sinc(double x){
        double y;
        if (abs(x) < 0.002) {
            //For small values of t use Taylor series approximation
            y = 1 - x^2/6 * (1 - x^2/20);
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

void circline(double s, double x0, double y0, double th0, double k,
            double& x, double& y, double& theta){

        x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2);
        y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2);
        theta = mod2pi(th0 + k * s); 
}

// Scale path finding problem into (-1,0) - (1,0) range 
void scaleToStandard(double x0, double y0, double th0,
                    double xf, double yf, double thf, double kmax, //this is int
                    double &scTh0, double &scThf, double &scKmax, double &lambda){
        
        // transform parameters (displacement, rotation, scaling factor)
        double dx = xf - x0; 
        double dy = yf - y0;
        double phi = std::atan2(dy, dx);
        lambda = hypot(dx, dy) / 2;

        // apply scaling and normalization (to orientation and curvature)
        scTh0 = mod2pi(th0 - phi);
        scThf = mod2pi(thf - phi);
        scKmax = kmax * lambda;
}

// Back to normal coordinates from (-1,0) - (1,0)
void scaleFromStandard(double lambda, double scS1, double scS2, double scS3,
                       double &s1, double &s2, double &s3){   
        s1 = scS1 * lambda; 
        s2 = scS2 * lambda;
        s3 = scS3 * lambda;

}

// Take as input the scaled parameters
bool check(double s1, double k0, double s2, double k1,
            double s3, double k2, double th0, double thf){

    // Negative length 
    if ((s1 < 0 ) && (s2 < 0) && (s3 < 0) )){
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

    return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.0e-10));

}

void dubinsRSL(){}

void dubinsRLR(){}

void dubinsLRL(){}
 