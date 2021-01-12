#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "visual_functions.hpp"

namespace student {

	//////// AUXILIARY FUNCTIONS ////////

	bool processGreen(const cv::Mat& img_hsv, cv::Mat& green_mask, bool DEBUG);
	bool processObstacles(const cv::Mat& img_orig, const cv::Mat& img_hsv, const double scale, std::vector<Polygon>& obstacle_list, bool DEBUG);	
	bool processBorders(const cv::Mat& img_orig, const cv::Mat& img_hsv, const double scale, std::vector<Polygon>& obstacle_list, bool DEBUG);	
	bool processGate(const cv::Mat& img_hsv, cv::Mat& green_mask, const double scale, Polygon& gate, bool DEBUG);
	bool processVictims(const cv::Mat& img_in, const cv::Mat& img_hsv, cv::Mat& green_mask, const double scale, 
			std::vector<std::pair<int,Polygon>>& victim_list, bool DEBUG);
	
	// For victim processing
	int findTemplateId(cv::Mat& processROI, std::vector<cv::Mat>& templates, bool DEBUG);
			cv::Mat rotate(cv::Mat src, double angle);



	//////////////// MAIN FUNCTION ////////////////

	bool student_processMap(const cv::Mat& img_in, const double scale,
			std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list,
			Polygon& gate, const std::string& config_folder, const bool DEBUG) {
	
		// Debug:
		//cv::Mat img_in = imread("/tmp/square_arena_real1.jpg", cv::IMREAD_COLOR);
		//cv::Mat img_in = imread("/tmp/numbers/ideal_unwarped.jpg", cv::IMREAD_COLOR);	 //for victims

		//////// VARIABLES ////////

  		// HSV matrix:
		cv::Mat img_hsv;
		// Green mask for double usage:
		cv::Mat green_mask;

    	//////// EXECUTION ////////

		// Convert color space from BGR to HSV
    	cv::cvtColor(img_in, img_hsv, cv::COLOR_BGR2HSV);

	    const bool found_obst = processObstacles(img_in, img_hsv, scale, obstacle_list, DEBUG);
	    if (!found_obst) {
			std::cerr << "ERROR IN METHOD <processObstacles> of student_processMap.cpp.\n" << std::flush;
	    }
	    else {
	    	std::cout << "\tOBSTACLES IDENTIFIED\n" << std::flush;
	    }    

	    const bool found_borders = processBorders(img_in, img_hsv, scale, obstacle_list, DEBUG);
	    if (!found_borders) {
			std::cerr << "ERROR IN METHOD <processBorders> of student_processMap.cpp.\n" << std::flush;
	    } else {
	    	std::cout << "\tARENA BORDERS IDENTIFIED\n" << std::flush;
	    }
	    
	    const bool proc_green = processGreen(img_hsv, green_mask, DEBUG);
	    if (!proc_green) {
	    	std::cerr << "ERROR IN METHOD <processGreen> of student_processMap.cpp.\n" << std::flush;
	    }
		else {
			std::cout << "\tGREEN MASK PROCESSED\n" << std::flush;
		}

		const bool found_gate = processGate(img_hsv, green_mask, scale, gate, DEBUG);
	    if (!found_gate) {
	    	std::cerr << "ERROR IN METHOD <processGate> of student_processMap.cpp.\n" << std::flush;
	    }
		else {
			std::cout << "\tGATE IDENTIFIED\n" << std::flush;
		}

	    const bool proc_vict = processVictims(img_in, img_hsv, green_mask, scale, victim_list, DEBUG);
	    if (!proc_vict) {
	    	std::cerr << "ERROR IN METHOD <processVictims> of student_processMap.cpp.\n" << std::flush;
	    }
		else {
			std::cout << "\tVICTIMS IDENTIFIED\n" << std::flush;
		}

		return found_obst && found_borders && proc_green && found_gate && proc_vict;

		//return found_borders;
	}


	bool processObstacles(const cv::Mat& img_in, const cv::Mat& img_hsv, const double scale, std::vector<Polygon>& obstacle_list, bool DEBUG) {

		//////// VARIABLES ////////

  		// Color masks:
		cv::Mat red_mask_low, red_mask_high, red_mask;
		
		// Red kernel:
		cv::Mat kernel_red = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2) + 1));

		//////// CONTOURS STRUCTURES ////////

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Point> approx_curve;

		// Debug variables:
		std::vector<std::vector<cv::Point>> contours_approx;
		cv::Mat contours_red, contours_black;

		//////// MASKS CREATION ////////

		cv::inRange(img_hsv, cv::Scalar(0, 35, 35), cv::Scalar(20, 255, 255), red_mask_low);
		cv::inRange(img_hsv, cv::Scalar(160, 35, 35), cv::Scalar(180, 255, 255), red_mask_high);

		// Combination of the 2 red masks:
		cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask);

		if (DEBUG) {
			char debug_1[] = "Red raw";
			cv::namedWindow(debug_1, 10);
	  		cv::imshow(debug_1, red_mask);
	  		cv::waitKey(30000);
	  		cv::destroyWindow(debug_1);
	  	}

		//////// MASKS FILTERING ////////

		cv::erode(red_mask, red_mask, kernel_red);
		cv::dilate(red_mask, red_mask, kernel_red);

		if (DEBUG) {
			char debug_2[] = "Red filtered";
			cv::namedWindow(debug_2, 10);
  			cv::imshow(debug_2, red_mask);
  			cv::waitKey(30000);
			cv::destroyWindow(debug_2);
		}

  		//////// CONTOURS PROCESSING - RED MASK ////////
	
		// Debug:
		contours_red = img_in.clone();

		cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (int i = 0; i < contours.size(); i++) {

			cv::approxPolyDP(contours[i], approx_curve, 8, true);

			Polygon scaled_contour_red;
			for (const auto& pt: approx_curve) {
				scaled_contour_red.emplace_back(pt.x/scale, pt.y/scale);
			}
			obstacle_list.push_back(scaled_contour_red);

			if (DEBUG) {
				std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
				contours_approx = {approx_curve};
				cv::drawContours(contours_red, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

				// Executed only once:
				if (i == contours.size() - 1) {
					std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
					char debug_3[] = "Original + red";
					cv::namedWindow(debug_3, 10);
					cv::imshow(debug_3, contours_red);
					cv::waitKey(30000);
					cv::destroyWindow(debug_3);
				}
			}
		}

		return true;
	}

	bool processBorders(const cv::Mat& img_orig, const cv::Mat& img_hsv, const double scale, std::vector<Polygon>& obstacle_list, bool DEBUG){
		
		//////// MASK AND FILTERING ////////

		cv::Mat black_mask;
		cv::inRange(img_hsv, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 40), black_mask);

		if (DEBUG) {
			char debug_1[] = "Black raw";
			cv::namedWindow(debug_1, 10);
	  		cv::imshow(debug_1, black_mask);
	  		cv::waitKey(30000);
	  		cv::destroyWindow(debug_1);
	  	}

		// Black kernel:
		cv::Mat kernel_black_1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2) + 1));
		cv::Mat kernel_black_2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size((1*2) + 1, (1*2) + 1));

		
		cv::erode(black_mask, black_mask, kernel_black_1);
		cv::dilate(black_mask, black_mask, kernel_black_1);
		//cv::erode(black_mask, black_mask, kernel_black_2);
		//cv::dilate(black_mask, black_mask, kernel_black_2);

		if (DEBUG) {
			char debug_2[] = "Black filtered";
			cv::namedWindow(debug_2, 10);
  			cv::imshow(debug_2, black_mask);
  			cv::waitKey(30000);
			cv::destroyWindow(debug_2);
		}

		//////// CONTOURS PROCESSING - BLACK MASK ////////

		//////// STRUCTURES ////////

		// image to draw black contours, for debug:
		cv::Mat contours_img = img_orig.clone(); 

		std::vector<std::vector<cv::Point>> contours, contours_approx, inner_approx;
		std::vector<cv::Point> approx_curve, inner_contours;		
		// bounding lines
	    std::vector<cv::Point> bound_top, bound_right, bound_left, bound_bottom; 

	    //////// PROCESSING ////////
		cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		for (int i = 0; i < contours.size(); i++) { // here there should be only one contour, the whole arena 

			const double area = cv::contourArea(contours[i]);
			if (area > 20000) { // the whole arena

				cv::approxPolyDP(contours[i], approx_curve, 7, true);
				std::cout << i << "th black contour area: " << area << std::endl;
				std::cout << i << "th black approx_curve size: " << approx_curve.size() << std::endl;
				
				// from the approximated curve, only take the extreme points

				std::vector<cv::Point> wholeArena;
				cv::Rect rect = cv::boundingRect(approx_curve);
				cv::Point minXY = rect.tl(); // top left
				cv::Point maxXY = rect.br() - cv::Point(1,1);

				std::cout << "Min point TL: " << minXY << " Max point BR: " << maxXY << std::endl;

				cv:: Point minXmaxY(minXY.x, minXY.y + rect.height -1); // Bottom left	
				cv:: Point maxXminY(minXY.x + rect.width-1, minXY.y); // Top right

				std::cout << "Min point BL: " << minXmaxY << " Max point TR: " << maxXminY << std::endl;

				wholeArena.emplace_back(minXY);
				wholeArena.emplace_back(minXmaxY);
				wholeArena.emplace_back(maxXY);
				wholeArena.emplace_back(maxXminY);

				int offset = 15;
				// Take outer vertices and compute inner ones
				for (int j = 0; j < wholeArena.size(); j++){
					cv::Point p = wholeArena[j];
					std::cout << "Contour point: " << p << std::endl;
				

			   		// TODO check values in simulator
			   		// NOTE: Beware the order in which the points are added 

					if (p.x < 400 && p.y < 300){
						cv::Point inner_top_l(p.x + offset, p.y + offset);
						inner_contours.emplace_back(inner_top_l);
			   			
			   			// Points on bounding line at the top(of the image as shown in tests), left
			   			bound_top.emplace_back(p);
			   			bound_top.emplace_back(inner_top_l);
			   			// Topmost points on bounding line on the left
			   			bound_left.emplace_back(p);
			   			bound_left.emplace_back(inner_top_l);
					}
					if (p.x > 400 && p.y < 300){
						cv::Point inner_top_r(p.x - offset, p.y + offset);
						inner_contours.emplace_back(inner_top_r);

			   			// Points on bounding line at the top(of the image as shown in tests), right
			   			bound_top.emplace_back(inner_top_r);
			   			bound_top.emplace_back(p);
			   			// Topmost points on bounding line on the right
			   			bound_right.emplace_back(inner_top_r);
			   			bound_right.emplace_back(p);

					}
					if (p.x > 400 && p.y > 300){
						cv::Point inner_bottom_r(p.x - offset, p.y - offset);
						inner_contours.emplace_back(inner_bottom_r);

			   			// Points on bounding line on the right, bottom
			   			bound_right.emplace_back(p);
			   			bound_right.emplace_back(inner_bottom_r);
			   			// Points on bounding line on the bottom, right
			   			bound_bottom.emplace_back(p);
			   			bound_bottom.emplace_back(inner_bottom_r);


					}
					if (p.x < 400 && p.y > 300){
						cv::Point inner_bottom_l(p.x + offset, p.y - offset);
						inner_contours.emplace_back(inner_bottom_l);

			   			// Points on bounding line on the bottom, left
			   			bound_bottom.emplace_back(inner_bottom_l);
			   			bound_bottom.emplace_back(p);
			   			// Bottom Points on bounding line on the left   			
			   			bound_left.emplace_back(inner_bottom_l);
			   			bound_left.emplace_back(p);

					}

				}
				//inner_approx = {inner_contours};	
				inner_approx.emplace_back(bound_top);
				inner_approx.emplace_back(bound_right);
				inner_approx.emplace_back(bound_bottom);
				inner_approx.emplace_back(bound_left);

				for (const auto& bounds: inner_approx) {
					Polygon obstacle;
					for (const auto& p: bounds) {
						obstacle.emplace_back(p.x/scale, p.y/scale);
					}
					obstacle_list.push_back(obstacle);
				}

//				Polygon scaled_contour_black;
//				for (const auto& pt: approx_curve) {
//					scaled_contour_black.emplace_back(pt.x/scale, pt.y/scale);
//				}
//				obstacle_list.push_back(scaled_contour_black);

				if (DEBUG) {
					std::cout << (i+1) << ") Black Contour size: " << contours[i].size() << std::endl;

					std::cout << (i+1) << ") Arena Contour size: " << wholeArena.size() << std::endl;
					//contours_approx = {approx_curve};
					//cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
					// Draw polygons as bounds 

					cv::drawContours(contours_img, inner_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
					
					// Debug: draw vertices
					int radiusCircle = 2;
					cv::Scalar colorCircle1(0,0,255);
	   				int thicknessCircle1 = 2;
					for (int k = 0; k < inner_contours.size(); k++){
						cv::circle(contours_img, inner_contours[k], radiusCircle, colorCircle1, thicknessCircle1);
					}

					// Executed only once:
					if (i == contours.size() - 1) {
						std::cout << "  Approximated black contour size: " << approx_curve.size() << std::endl;
						char debug_3[] = "Original + black";
						cv::namedWindow(debug_3, 10);
						cv::imshow(debug_3, contours_img);
						cv::waitKey(30000);
						cv::destroyWindow(debug_3);
					}
				}
			}
		}
		return true;
	}


	bool processGreen(const cv::Mat& img_hsv, cv::Mat& green_mask, bool DEBUG) {
		
		//////// VARIABLES ////////

		// Green kernel:
		cv::Mat kernel_green = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2) + 1));

		//////// MASK CREATION ////////
		
		cv::inRange(img_hsv, cv::Scalar(40, 25, 25), cv::Scalar(80, 255, 255), green_mask);

		if (DEBUG) {
			char debug_7[] = "Green raw";
			cv::namedWindow(debug_7, 10);
	  		cv::imshow(debug_7, green_mask);
	  		cv::waitKey(30000);
	  		//cv::destroyWindow(debug_7);
	  	}

		//////// MASK FILTERING ////////

		cv::erode(green_mask, green_mask, kernel_green);
		cv::dilate(green_mask, green_mask, kernel_green);

		if (DEBUG) {
			char debug_8[] = "Green filtered";
			cv::namedWindow(debug_8, 10);
  			cv::imshow(debug_8, green_mask);
			cv::waitKey(30000);
			//cv::destroyWindow(debug_8);
			cv::destroyAllWindows();
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

		// Debug:
		contours_green = img_hsv.clone();

		bool gate_found = false;
		for( auto& contour : contours) {
			const double area = cv::contourArea(contour);

			if (area > 500) {
				cv::approxPolyDP(contour, approx_curve, 8, true);

				if(approx_curve.size() != 4) continue;

				if (DEBUG) {
					std::cout << "AREA " << area << std::endl;
					std::cout << "SIZE: " << contours.size() << std::endl;
				}

				for (const auto& pt: approx_curve) {
					gate.emplace_back(pt.x/scale, pt.y/scale);
				}
				gate_found = true;

				if (DEBUG) {
					std::cout << "N. contours: " << contours.size() << std::endl;
					contours_approx = {approx_curve};
					cv::drawContours(contours_green, contours_approx, -1, cv::Scalar(0,170,220), 4, cv::LINE_AA);

					std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
					char debug_9[] = "Original + gate";
					cv::namedWindow(debug_9, 10);
					cv::imshow(debug_9, contours_green);
					cv::waitKey(30000);
					cv::destroyWindow(debug_9);
				}

				break;
			}
	    }

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
			char debug_10[] = "Inverted";
			char debug_11[] = "Numbers";
			cv::namedWindow(debug_10, 10);
			cv::imshow(debug_10, filtered);
			cv::namedWindow(debug_11, 10);
			cv::imshow(debug_11, green_mask_inv);
			cv::waitKey(30000);
			cv::destroyWindow(debug_10);
			cv::destroyWindow(debug_11);
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

		cv::destroyAllWindows();
		return true;
	}

	// Processes an image against the number templates, and returns the id of the best match
	int findTemplateId(cv::Mat& ROI_rect, std::vector<cv::Mat>& templates, bool DEBUG){

		cv::resize(ROI_rect, ROI_rect, cv::Size(200,200));
		cv::threshold(ROI_rect, ROI_rect, 100, 255, 0);		
   		cv::GaussianBlur(ROI_rect, ROI_rect, cv::Size(5, 5), 2, 2);

		if (DEBUG) {/*
			char debug_12[] = "ROI";
			cv::namedWindow(debug_12, 10);
			cv::imshow(debug_12, ROI_rect);
			cv::waitKey(30000);
			cv::destroyWindow(debug_12);*/
		}	

		// Image from simulator is unwarped, so flip the rectangle with number along y axis 
		cv::Mat flippedROI;
		cv::flip(ROI_rect, flippedROI, 1);

		if (DEBUG) {
			char debug_13[] = "Flipped";
			cv::namedWindow(debug_13, 10);
			cv::imshow(debug_13, flippedROI);
			cv::waitKey(1500);
			//cv::destroyWindow(debug_13);
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
				char debug_14[] = "ROI rotated";
				cv::namedWindow(debug_14, 10);
				cv::imshow(debug_14, rotated);
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

			if (DEBUG) {
				std::cout << "Max score, id for rotated image: " << max_score << "," << max_id + 1 << std::endl;
				cv::destroyAllWindows();
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
