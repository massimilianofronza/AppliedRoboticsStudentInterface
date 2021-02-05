#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "visual_functions.hpp"

#include <sstream>

namespace student{

	/**
	* Student implementation of the findRobot() function. 
	* It processes the blue area of the arena, identifies the triangle which represents 
	* the robot, processes it to find its center (x,y) and orientation (theta). 
	* If DEBUG is true, more information about the process is printed and images are shown. 
	*/
	bool student_findRobot(const cv::Mat& img_in, const double scale, 
			Polygon& triangle, double& x, double& y, 
			double& theta, const std::string& config_folder, const bool DEBUG) {

		// Convert color space from BGR to HSV
		cv::Mat hsv_img;
		cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);    

		if (DEBUG) {
			char debug_1[] = "Arena";
			char debug_2[] = "Arena hsv";
			cv::namedWindow(debug_1, 10);
			cv::imshow(debug_1, img_in);
			cv::namedWindow(debug_2 ,10);
			cv::imshow(debug_2, hsv_img);
			cv::waitKey(1500); 
			//cv::destroyWindow(debug_1);
			//cv::destroyWindow(debug_2);
		}

		// Prepare blue mask, with HSV values that best worked for real images
		cv::Mat blue_mask;
		cv::inRange(hsv_img, cv::Scalar(90, 70, 35), cv::Scalar(140, 255, 255), blue_mask);	

		// Erosion and delatation to get rid of noise and to make robot more clear
		cv::Mat er_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1,-1));
		cv::erode(blue_mask, blue_mask, er_kernel);

		cv::Mat dil_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1,-1));
		cv::dilate(blue_mask, blue_mask, dil_kernel);

		if (DEBUG) {
			char debug_3[] = "Blue filter clean";
			cv::namedWindow(debug_3, 10);
			cv::imshow(debug_3, blue_mask);
			cv::waitKey(10);
			//cv::destroyWindow(debug_3);
		}

		// Process blue mask and find countours
		std::vector<std::vector<cv::Point>> contours, approximated;
		std::vector<cv::Point> approx_curve;
		cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);		

		if (DEBUG){ std::cout << "N. contours: " << contours.size() << std::endl; }


		bool found = false;

		for (int i = 0; i<contours.size(); i++){

			// Approximates the found shapes to closed shapes with less vertices
			cv::approxPolyDP(contours[i], approx_curve, 7, true);
			approximated = {approx_curve};
	
			// If it is not a triangle, continue searching 
			if (approx_curve.size() != 3) continue;

			double A = cv::contourArea(approx_curve);
			
			if (DEBUG){
				// double check area of robot
				std::cout << "Area of robot: " << A << std::endl;	
			}	
		    
		    // If it is an area too small or too big, continue searching
		    if (A < 300 || A > 3000) continue;

			// There should be only 1 blue object, the robot
			found = true;	
			break;
		}

		// Now that robot is found, assign the shape to the triangle
		// of the input and compute centroid
		if (found){

			// centroid
			double center_x=0, center_y=0;	

			// approximated curve has 3 vertices
			for (const auto& vertex: approx_curve){

				// Assign the vertices to the triangle
				double vx = vertex.x/scale;
				double vy = vertex.y/scale;
				triangle.emplace_back(vx, vy);	
		
				center_x += vx;
				center_y += vy;

			}

			center_x /= triangle.size();
			center_y /= triangle.size();

			if (DEBUG) {
				std::cout << "center_x = " << center_x << std::endl; 
				std::cout << "center_y = " << center_y << std::endl; 
				std::cout << "Triangle size: " << triangle.size() << std::endl;
			}

			// rotation angle theta is computed based on displacement
			// of centroid wrt major distance vertex (origin)
			Point origin;
			double distance = 0;
			for (auto& vertex: triangle) {
				double dx = vertex.x - center_x;
				double dy = vertex.y - center_y;
				double curr_distance = dx*dx + dy*dy;

				if (curr_distance > distance){
					distance = curr_distance;
					origin = vertex;
				}
			}
			double dx = center_x - origin.x;
			double dy = center_y - origin.y;
	
			// Output variables
			x = center_x;
			y = center_y;
			double theta = std::atan2(dy, dx);

			if (DEBUG) { std::cout << "Rotation angle: " << theta << std::endl; }

		}
		//std::cout << "\t\tROBOT FOUND" << std::endl; 
		return found;
	}
}
