#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "student_functions.hpp"

namespace student {

	//////// AUXILIARY FUNCTIONS ////////

	bool processGreen(const cv::Mat& img_hsv, cv::Mat& green_mask, bool DEBUG);
	bool processObstacles(const cv::Mat& img_hsv, const double scale, std::vector<Polygon>& obstacle_list, bool DEBUG);
	bool processGate(const cv::Mat& img_hsv, cv::Mat& green_mask, const double scale, Polygon& gate, bool DEBUG);
	bool processVictims(const cv::Mat& img_in, const cv::Mat& img_hsv, cv::Mat& green_mask, const double scale, 
			std::vector<std::pair<int,Polygon>>& victim_list, bool DEBUG);
	
	// For victim processing
	int findTemplateId(cv::Mat& processROI, std::vector<cv::Mat>& templates, bool DEBUG);
	cv::Mat rotate(cv::Mat src, double angle);



	//////////////// MAIN FUNCTION ////////////////

	bool student_processMap(const cv::Mat& img_in, const double scale,
			std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list,
			Polygon& gate, const std::string& config_folder) {
	
//cv::Mat img_in = imread("/tmp/square_arena_real1.jpg", cv::IMREAD_COLOR);
//cv::Mat img_in = imread("/tmp/numbers/ideal_unwarped.jpg", cv::IMREAD_COLOR);	 //for victims

		//////// VARIABLES ////////

		// Debug variable to execute more code:
		bool DEBUG = true;
  		// HSV matrix:
		cv::Mat img_hsv;
		// Green mask for double usage:
		cv::Mat green_mask;

    	//////// EXECUTION ////////

		// Convert color space from BGR to HSV
    	cv::cvtColor(img_in, img_hsv, cv::COLOR_BGR2HSV);

	    const bool found_obst = processObstacles(img_hsv, scale, obstacle_list, DEBUG);
	    if (!found_obst) {
			std::cerr << "ERROR IN METHOD <processObstacles> of student_processMap.cpp.\n" << std::flush;
	    }
	    const bool proc_green = processGreen(img_hsv, green_mask, DEBUG);
	    if (!proc_green) {
	    	std::cerr << "ERROR IN METHOD <processGreen> of student_processMap.cpp.\n" << std::flush;
	    }
		const bool found_gate = processGate(img_hsv, green_mask, scale, gate, DEBUG);
	    if (!found_gate) {
	    	std::cerr << "ERROR IN METHOD <processGate> of student_processMap.cpp.\n" << std::flush;
	    }
	    const bool proc_vict = processVictims(img_in, img_hsv, green_mask, scale, victim_list, DEBUG);
	    if (!proc_vict) {
	    	std::cerr << "ERROR IN METHOD <processVictims> of student_processMap.cpp.\n" << std::flush;
	    }

	    return found_obst && proc_green && found_gate && proc_vict;
	}


	bool processObstacles(const cv::Mat& img_hsv, const double scale, std::vector<Polygon>& obstacle_list, bool DEBUG) {

		//////// VARIABLES ////////

  		// Color masks:
		cv::Mat red_mask_low, red_mask_high, red_mask, black_mask;
// Fossero stati solo cerchi avrei usato ELLIPSE, ma essendoci anche il gate da identificare ho preferito così
		
		// Red kernel:
		cv::Mat kernel_red = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2) + 1));
		// Black kernel:
		cv::Mat kernel_black_1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((5*2) + 1, (5*2) + 1));
		cv::Mat kernel_black_2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size((1*2) + 1, (1*2) + 1));

		//////// CONTOURS STRUCTURES ////////

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Point> approx_curve;

		// Debug variables:
		std::vector<std::vector<cv::Point>> contours_approx;
		cv::Mat contours_red, contours_black;

		//////// MASKS CREATION ////////

		cv::inRange(img_hsv, cv::Scalar(0, 35, 35), cv::Scalar(20, 255, 255), red_mask_low);
		cv::inRange(img_hsv, cv::Scalar(160, 35, 35), cv::Scalar(180, 255, 255), red_mask_high);
	
		cv::inRange(img_hsv, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 40), black_mask);

		// Combination of the 2 red masks:
// Four different ways to combine together the two binary masks:
		cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask);
//red_mask = red_mask_low | red_mask_high;
//red_mask = red_mask_low + red_mask_high;
//cv::bitwise_or(red_mask_low, red_mask_high, red_mask);

		if (DEBUG) {
			cv::namedWindow("Red raw", 10);
	  		cv::imshow("Red raw", red_mask);
			cv::namedWindow("Black raw", 10);
	  		cv::imshow("Black raw", black_mask);
	  		cv::waitKey(0);
	  	}

		//////// MASKS FILTERING ////////

		cv::erode(red_mask, red_mask, kernel_red);
		cv::dilate(red_mask, red_mask, kernel_red);

		cv::dilate(black_mask, black_mask, kernel_black_1);
		cv::erode(black_mask, black_mask, kernel_black_1);
		cv::erode(black_mask, black_mask, kernel_black_2);
		cv::dilate(black_mask, black_mask, kernel_black_2);

		if (DEBUG) {
			cv::namedWindow("Red filtered", 10);
  			cv::imshow("Red filtered", red_mask);
			cv::namedWindow("Black filtered", 10);
  			cv::imshow("Black filtered", black_mask);
  			cv::waitKey(0);
  		}

  		//////// CONTOURS IDENTIFICATION ////////
	
contours_red = img_hsv.clone();
		cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
cv::drawContours(contours_red, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
//std::cout << "N. contours: " << contours.size() << std::endl;
		for (int i = 0; i < contours.size(); i++) {
//std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
			cv::approxPolyDP(contours[i], approx_curve, 8, true);
contours_approx = {approx_curve};
cv::drawContours(contours_red, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

			Polygon scaled_contour_red;
			for (const auto& pt: approx_curve) {
				scaled_contour_red.emplace_back(pt.x/scale, pt.y/scale);
			}
			obstacle_list.push_back(scaled_contour_red);

//std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
		}

cv::imshow("Original", contours_red);
cv::waitKey(0);

contours_black = img_hsv.clone();
		cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
cv::drawContours(contours_black, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
//std::cout << "N. contours: " << contours.size() << std::endl;
		for (int i = 0; i < contours.size(); i++) {
//std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
			cv::approxPolyDP(contours[i], approx_curve, 8, true);
contours_approx = {approx_curve};
cv::drawContours(contours_black, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

			Polygon scaled_contour_black;
			for (const auto& pt: approx_curve) {
				scaled_contour_black.emplace_back(pt.x/scale, pt.y/scale);
			}
			obstacle_list.push_back(scaled_contour_black);

//std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
		}

cv::imshow("Original", contours_black);
cv::waitKey(0);

		return true;
	}


	bool processGreen(const cv::Mat& img_hsv, cv::Mat& green_mask, bool DEBUG) {
		
		//////// VARIABLES ////////

		// Green kernel:
		cv::Mat kernel_green = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2) + 1));

		//////// MASK CREATION ////////
		
		cv::inRange(img_hsv, cv::Scalar(40, 25, 25), cv::Scalar(80, 255, 255), green_mask);

		if (DEBUG) {
			cv::namedWindow("Green raw", 10);
	  		cv::imshow("Green raw", green_mask);
	  		cv::waitKey(0);
	  	}

		//////// MASK FILTERING ////////

		cv::erode(green_mask, green_mask, kernel_green);
		cv::dilate(green_mask, green_mask, kernel_green);

		if (DEBUG) {
			cv::namedWindow("Green filtered", 10);
  			cv::imshow("Green filtered", green_mask);
			cv::waitKey(0);
  		}

		return true;
	}


	bool processGate(const cv::Mat& img_hsv, cv::Mat& green_mask, const double scale, Polygon& gate, bool DEBUG) {
	
		//////// STRUCTURES ////////

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Point> approx_curve;
		
// Debug variables
std::vector<std::vector<cv::Point>> contours_approx;
cv::Mat contours_green;

  		//////// CONTOURS IDENTIFICATION ////////

		cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		//////// GATE IDENTIFICATION ////////

contours_green = img_hsv.clone();
cv::drawContours(contours_green, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
std::cout << "N. contours: " << contours.size() << std::endl;
		
		bool gate_found = false;
std::cout << "N. contours: " << contours.size() << std::endl;
		for( auto& contour : contours) {
			const double area = cv::contourArea(contour);
std::cout << "AREA " << area << std::endl;
std::cout << "SIZE: " << contours.size() << std::endl;
			if (area > 500) {
				cv::approxPolyDP(contour, approx_curve, 8, true);

				if(approx_curve.size() != 4) continue;	// prima era !=4, così se per sfiga il gate dovesse essere un po' diverso, non avremmo problemi

contours_approx = {approx_curve};
cv::drawContours(contours_green, contours_approx, -1, cv::Scalar(0,170,220), 4, cv::LINE_AA);

				for (const auto& pt: approx_curve) {
					gate.emplace_back(pt.x/scale, pt.y/scale);
				}
				gate_found = true;
				break;
			}
std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
	    }

cv::imshow("Original", contours_green);
cv::waitKey(0);

    	return gate_found;
	}


	bool processVictims(const cv::Mat& img_in, const cv::Mat& img_hsv, cv::Mat& green_mask, const double scale, 
			std::vector<std::pair<int,Polygon>>& victim_list, bool DEBUG) {
   
		//////// STRUCTURES ////////

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Point> approx_curve;
		
		// Debug variables
		std::vector<std::vector<cv::Point>> contours_approx;
		cv::Mat contours_green;

		//////// CONTOURS IDENTIFICATION ////////

		cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		if (DEBUG){
			std::cout << "N. contours: " << contours.size() << std::endl;
		}

		//////// VICTIMS IDENTIFICATION ////////
		
		////// Load template images for victim numbers /////
		
		std::string folder = "/home/ubuntu/workspace/project/templates/";
		std::vector<cv::Mat> templates;
		for (int i = 1; i <= 5; i++){
			cv::Mat num = cv::imread(folder + std::to_string(i) + ".png");	
			if (!num.data) {
				printf("Error opening template!\n");
			}
			templates.emplace_back(num);
		}

		// Invert mask for number processing

		cv::Mat green_mask_inv, filtered(img_in.rows, img_in.cols, CV_8UC3, cv::Scalar(255,255,255));
		cv::bitwise_not(green_mask, green_mask_inv); 
		img_in.copyTo(filtered, green_mask_inv);
		if (DEBUG) {
			cv::namedWindow("Inverted",10);
			cv::imshow("Inverted", filtered);

			/*cv::namedWindow("Numbers", 10);
			cv::imshow("Numbers", green_mask_inv);
			*/
			cv::waitKey(100);

		}
		
		// Find and process bounding rectangles of victims
		std::vector<cv::Rect> boundRect(contours.size());
 		for (int i=0; i<contours.size(); ++i)
        	{
			std::cout << "Processing contour " << i << std::endl;
		  	const double area = cv::contourArea(contours[i]);

			if(area < 500) continue;   // remove false positives

			approxPolyDP(contours[i], approx_curve, 10, true);
			if(approx_curve.size() < 6) continue;
		
			// debug
			//contours_approx = {approx_curve};
			//drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

			// find bounding rectangle for victims
			boundRect[i] = cv::boundingRect(cv::Mat(approx_curve));
			cv::Mat ROI_rect(filtered, boundRect[i]);
			if(ROI_rect.empty()) continue;

			// Find best matching template id
			int maxId = findTemplateId(ROI_rect, templates, DEBUG);

			if (DEBUG){
				std::cout << "Best fitting template: " << maxId << std::endl;
			}

			// Add victim to list, with corresponding id
			Polygon circle;
			for (const auto& pt: approx_curve) {
				double x = pt.x/scale;
				double y = pt.y/scale;
		       		circle.emplace_back(x, y);
				//std::cout << "Circle point: (x,y) = (" << x << "," << y << ")" << std::endl; 
		      	}
		
			victim_list.push_back({maxId, circle});
	
		}

		if(DEBUG){
			std::cout << "N. victims: " << victim_list.size() << std::endl;
		}

		//cv::destroyAllWindows();
		return true;
	}

	// Processes an image against the number templates, and returns the id of the best match
	int findTemplateId(cv::Mat& ROI_rect, std::vector<cv::Mat>& templates, bool DEBUG){

		cv::resize(ROI_rect, ROI_rect, cv::Size(200,200));
		cv::threshold(ROI_rect, ROI_rect, 100, 255, 0);		
   		cv::GaussianBlur(ROI_rect, ROI_rect, cv::Size(5, 5), 2, 2);

		if (DEBUG){		
			/*	
			cv::namedWindow("ROI", 10);
			cv::imshow("ROI", ROI_rect);
			cv::waitKey(10);
			*/
		}	

		// Image from simulator is unwarped, so flip the rectangle with number along y axis 
		cv::Mat flippedROI;
		cv::flip(ROI_rect, flippedROI, 1);

		if (DEBUG){
			cv::namedWindow("Flipped", 10);
			cv::imshow("Flipped", flippedROI);
			cv::waitKey(1500);
 		}

		// Variables for template matching
		double max_score = 0;
		int max_id = -1; 

		// Rotation angles for images from simulator
		int angles [] = {0, 45, 90, 135, 180, 225, 270, 315};
		int no_angles = sizeof(angles)/sizeof(angles[0]);

		// Rotate the image by the above defined angles
		for (int i = 0; i < no_angles; i++){

			cv::Mat rotated;
			rotated = rotate(flippedROI, angles[i]);

			if (DEBUG){
				cv::namedWindow("ROI rotated", 10);
				cv::imshow("ROI rotated", rotated);
				cv::waitKey(1500);
			}
			
			// Check which template fits best
			for (int j = 0; j < templates.size(); j++){

				cv::Mat result;
				cv::matchTemplate(rotated, templates[j], result, cv::TM_CCOEFF);

				double score;
				cv::minMaxLoc(result, nullptr, &score);

				//std::cout << "Score for template " << (j+1) << ": " << score << std::endl;
				if (score > max_score){
					max_score = score;
					max_id = j;
				}
			}

			if(DEBUG){
				std::cout << "Max score, id for rotated image: " << max_score << "," << max_id + 1 << std::endl;
			}
	
		}
		
		
		// templates are from 1 to 5, not 0 - 4
		return max_id + 1;
	}

	// Function to rotate a ROI by a certain angle    
	cv::Mat rotate(cv::Mat src, double angle) {

		// Output
		cv::Mat dst; 

		// Anchor from where to rotate (center of image)    
		cv::Point2f pt(src.cols/2., src.rows/2.); 
             
		 //Mat object for storing after rotation
		cv::Mat r = getRotationMatrix2D(pt, angle, 1.0);   
  
		//apply an affine transforation to the image.
		cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));  

		return dst;      
	}

}
