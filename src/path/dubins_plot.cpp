#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "dubins_functions.hpp"

namespace student {

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
            temp = getNextConfig(arc.currentConf, arc.k, s);
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

}