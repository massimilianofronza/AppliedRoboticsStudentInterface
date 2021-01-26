#include "dubins_functions.hpp"
#include "collision_functions.hpp"


namespace student {

	// If the image is scaled incorrectly, change the fact variable
	void plotLines(bool res, double x1, double y1, double x2, double y2,
							 double x3, double y3, double x4, double y4) {

		// The resulting image
		cv::Mat image = cv::Mat::zeros(PLOT_Y_SIZE, PLOT_X_SIZE, CV_8UC3);

		// Setting up a white background
		image.setTo(cv::Scalar(255, 255, 255));

		// Defining a scaling factor and an y-offset
		double fact = 10;
		double off = 30;

		// Defining points
		cv::Point s1 = cv::Point(x1*fact, y1*fact + off);
		cv::Point e1 = cv::Point(x2*fact, y2*fact + off);
		cv::Point s2 = cv::Point(x3*fact, y3*fact + off);
		cv::Point e2 = cv::Point(x4*fact, y4*fact + off);

		// Plot the lines
		cv::line(image, s1, e1, cv::Scalar(255, 0, 0), 3, cv::LINE_8);
		cv::line(image, s2, e2, cv::Scalar(0, 255, 0), 3, cv::LINE_8);
		
		// Show the collision outcome in the image title
		if (res) {
			cv::namedWindow("Collision detection returned TRUE", cv::WINDOW_AUTOSIZE);
			cv::imshow("Collision detection returned TRUE", image);
		}
		else {
			cv::namedWindow("Collision detection returned FALSE", cv::WINDOW_AUTOSIZE);
			cv::imshow("Collision detection returned FALSE", image);
		}
        
        cv::waitKey(0);
	}

}