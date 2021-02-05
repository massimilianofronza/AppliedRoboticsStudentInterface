#include "dubins_functions.hpp"

namespace student {


    // -------------------------------------------
    // PLOT FUNCTIONS
    // -------------------------------------------

    // Plot of the arc's points on an empty image
    void plot(cv::Mat image, double **pts_1, cv::Scalar c1, 
                             double **pts_2, cv::Scalar c2, 
                             double **pts_3, cv::Scalar c3) {
        
        // Linking all consecutive points with a line
        for (int r=0; r<99; r++) {
            cv::line(image, cv::Point(pts_1[r][0], pts_1[r][1]), 
                            cv::Point(pts_1[r+1][0], pts_1[r+1][1]), c1, 4, cv::LINE_8);
            
            cv::line(image, cv::Point(pts_2[r][0], pts_2[r][1]), 
                            cv::Point(pts_2[r+1][0], pts_2[r+1][1]), c2, 4, cv::LINE_8);
            
            cv::line(image, cv::Point(pts_3[r][0], pts_3[r][1]), 
                            cv::Point(pts_3[r+1][0], pts_3[r+1][1]), c3, 4, cv::LINE_8);
        }

    }

    // Plot an arc (circular or straight)
    void get_arc_points(double **pts, dubinsArc arc, int npts) {
        configuration temp;
        
        // Obtaining points
        for (int j=0; j<npts; j++) {

            double s = arc.len / npts * j;
            temp = getNextConfig(arc.currentConf, arc.k, s);
            pts[j][0] = (temp.x + 2.0) * 100.0;     // TODO these offset may be removed
            pts[j][1] = DUB_PLOT_Y_SIZE - ((temp.y + 3.0) * 100.0);

            // Correctness check
            if ((pts[j][0] < 0) or (pts[j][1]) < 0) {
          	    std::cerr << "________ERROR IN METHOD <get_arc_points> of dubins_plot.cpp: negative point(s).________\n";
                exit(-1);
            }
        }

    }

    // Plot a Dubins curve
    void plot_dubins(std::vector<dubinsCurve> curves) {
        // The resulting image
        cv::Mat image = cv::Mat::zeros(DUB_PLOT_Y_SIZE, DUB_PLOT_X_SIZE, CV_8UC3);

        // Setting up a white background
        image.setTo(cv::Scalar(255, 255, 255));

        // Color of the arcs
        cv::Scalar c1 = cv::Scalar(255, 0, 0);
        cv::Scalar c2 = cv::Scalar(0, 255, 0);
        cv::Scalar c3 = cv::Scalar(0, 0, 255);

        // Three set of points, one for each arc
        double **pts_1;
        double **pts_2;
        double **pts_3;

        int npts = 100;

        // Allocating the points matrix, with 2 standing for x and y coordinates
        pts_1 = new double *[npts];
        pts_2 = new double *[npts];
        pts_3 = new double *[npts];

        for(int i=0; i<npts; i++) {
            pts_1[i] = new double[2];
            pts_2[i] = new double[2];
            pts_3[i] = new double[2];
        }

        // Plot each curve on the same image
        for (int i=0; i<curves.size(); i++) {
        
            // Get points of the three arcs
            get_arc_points(pts_1, curves[i].a1, npts);
            get_arc_points(pts_2, curves[i].a2, npts);
            get_arc_points(pts_3, curves[i].a3, npts);
        
            plot(image, pts_1, c1, pts_2, c2, pts_3, c3);
        }

        // Free the memory
        for (int i=0; i<npts; i++) {
            delete[] pts_1[i];
            delete[] pts_2[i];
            delete[] pts_3[i];
        }

        // Show the final image
        cv::namedWindow("Dubins curve", cv::WINDOW_AUTOSIZE);
        cv::imshow("Dubins curve", image);
        cv::waitKey(0);
        cv::destroyWindow("Dubins curve");

    }

}