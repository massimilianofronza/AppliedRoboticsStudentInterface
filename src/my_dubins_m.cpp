#include <cmath>
#include <string>

#define PI 3.14159265

// -------------------------------------------
// FUNCTIONS DECLARATIONS
// -------------------------------------------

bool dubinsLSL(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3);

bool dubinsRSR(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3);

bool dubinsLSR(double sc_th0, double sc_thf, int sc_Kmax, 
                double& sc_s1, double& sc_s2, double& sc_s3);

double scaleFromStandard(double lambda, double& sc_s1, double& sc_s2, double& sc_s3);



void scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3,
                double& s1, double& s2, double& s3) {
    s1 = sc_s1 * lambda;
    s2 = sc_s2 * lambda;
    s3 = sc_s3 * lambda;
}


// -------------------------------------------
// 6 POTENTIAL OPTIMAL DUBINS CURVES
// -------------------------------------------

bool dubinsLSL(double sc_th0, double sc_thf, int sc_Kmax, 
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

bool dubinsRSR(double sc_th0, double sc_thf, int sc_Kmax, 
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


bool dubinsLSR(double sc_th0, double sc_thf, int sc_Kmax, 
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
