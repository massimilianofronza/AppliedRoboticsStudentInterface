#include "collision_functions.hpp"

namespace student {

	/**
	 * Function to plot a possible collision between two lines
	 * If the image is scaled incorrectly, change the fact variable.
	 */
	void plotXLine(bool res, double x1, double y1, double x2, double y2,
							  double x3, double y3, double x4, double y4) {

		// The resulting image
		cv::Mat image = cv::Mat::zeros(COLL_PLOT_Y_SIZE, COLL_PLOT_X_SIZE, CV_8UC3);

		// Setting up a white background
		image.setTo(cv::Scalar(255, 255, 255));

		// Defining a scaling factor and an y-offset
		double fact = 50;
		double off = 3;

		// Defining points
		cv::Point s1 = cv::Point(x1*fact, COLL_PLOT_Y_SIZE - (y1 + off) *fact);
		cv::Point e1 = cv::Point(x2*fact, COLL_PLOT_Y_SIZE - (y2 + off) *fact);
		cv::Point s2 = cv::Point(x3*fact, COLL_PLOT_Y_SIZE - (y3 + off) *fact);
		cv::Point e2 = cv::Point(x4*fact, COLL_PLOT_Y_SIZE - (y4 + off) *fact);

		// Plot the lines
		cv::line(image, s1, e1, cv::Scalar(255, 0, 0), 3, cv::LINE_8);
		cv::line(image, s2, e2, cv::Scalar(0, 255, 0), 3, cv::LINE_8);
		
		// Show the collision outcome in the image title
		if (res) {
			cv::namedWindow("Collision_detection_returned_TRUE", cv::WINDOW_AUTOSIZE);
			cv::imshow("Collision_detection_returned_TRUE", image);
			cv::waitKey(0);
			cv::destroyWindow("Collision_detection_returned_TRUE");
		}
		else {
			cv::namedWindow("Collision_detection_returned_FALSE", cv::WINDOW_AUTOSIZE);
			cv::imshow("Collision_detection_returned_FALSE", image);
			cv::waitKey(0);
			cv::destroyWindow("Collision_detection_returned_FALSE");
		}
	}


	/**
	 * Function to plot a possible collision between two lines
	 * If the image is scaled incorrectly, change the fact variable.
	 */
	void plotXCircle(bool res, double x1, double y1, double x2, double y2,
							   double xC, double yC, double radius, 
							   cv::Point a1, cv::Point a2, double verse) {

		// The resulting image
		cv::Mat image = cv::Mat::zeros(COLL_PLOT_Y_SIZE, COLL_PLOT_X_SIZE, CV_8UC3);

		// Setting up a white background
		image.setTo(cv::Scalar(255, 255, 255));

		// Defining a scaling factor and an y-offset
		double fact = 30;
		double off = 6;

		// Defining points
		cv::Point s1 = cv::Point((x1 + off)*fact, COLL_PLOT_Y_SIZE - ((y1 + off)*fact));
		cv::Point e1 = cv::Point((x2 + off)*fact, COLL_PLOT_Y_SIZE - ((y2 + off)*fact));
		cv::Point cC = cv::Point((xC + off)*fact, COLL_PLOT_Y_SIZE - ((yC + off)*fact));
		int r = radius*fact;
		a1.x = (a1.x + off)*fact;
		a1.y = COLL_PLOT_Y_SIZE - ((a1.y + off)*fact);
		a2.x = (a2.x + off)*fact;
		a2.y = COLL_PLOT_Y_SIZE - ((a2.y + off)*fact);

		/// Plot line, circle and the two arc points as circles, the smallest as 
		/// the starting one.
		cv::circle(image, cC, r, cv::Scalar(0, 255, 0), 3, cv::LINE_8);
		cv::circle(image, a1, 1, cv::Scalar(0, 0, 255), 6, cv::LINE_8);
		cv::circle(image, a2, 2, cv::Scalar(0, 0, 255), 14, cv::LINE_8);
		cv::line(image, s1, e1, cv::Scalar(255, 0, 0), 3, cv::LINE_8);

		if (verse > 0) {		/// Left turn
			cv::putText(image, "LEFT", a1, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,0), 2.3);
		}
		else if (verse < 0) {	/// Left turn
			cv::putText(image, "RIGHT", a1, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,0), 2.3);
		}
		else {
			std::cerr << "________ERROR IN METHOD <plotXCircle> of collision_plot.cpp: a straight line was passed instead of an arc.________\n";
			exit(-1);
		}
		
		// Show the collision outcome in the image title
		if (res) {
			cv::namedWindow("Collision_detection_returned_TRUE", cv::WINDOW_AUTOSIZE);
			cv::imshow("Collision_detection_returned_TRUE", image);
			cv::waitKey(0);
			cv::destroyWindow("Collision_detection_returned_TRUE");
		}
		else {
			cv::namedWindow("Collision_detection_returned_FALSE", cv::WINDOW_AUTOSIZE);
			cv::imshow("Collision_detection_returned_FALSE", image);
			cv::waitKey(0);
			cv::destroyWindow("Collision_detection_returned_FALSE");
		}		
	}

}