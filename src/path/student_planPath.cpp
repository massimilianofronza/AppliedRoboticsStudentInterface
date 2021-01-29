#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "visual_functions.hpp"
#include "path_functions.hpp"
#include "dubins_functions.hpp"

namespace student{

	const bool DEBUG = true;

	bool student_planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
                 const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
                 const float x, const float y, const float theta, Path& path,
                 const std::string& config_folder){


		// TODO these two global variables don't work
		std::cout << "Inside student_planPath, size of offsetted_obstacles: " << offsetted_obstacles.size() << std::endl; // here, should be != 0
		std::cout << "SCALE " << SCALE << std::endl;
		// debug image
		cv::Mat image = cv::Mat::zeros(300,300, CV_8UC3);

		// Variables to keep victims and their centroid sorted 
		std::vector<std::pair<int, int>> sorted_index;
		std::vector<Point> victim_centers;

		// Sorted list of points to be visited
		std::vector<Point> point_list;

		// Process victims, compute their centroid and populate the above variables
		for (int i = 0; i < victim_list.size(); i++){
			int id = victim_list[i].first;
			Polygon victim = victim_list[i].second;

			//std::cout << i << "th victim id: " << id << std::endl;

			double cx = 0, cy = 0;
			for (const auto& pt: victim){

			//	std::cout << "\tPoint: (" << pt.x << "," << pt.y << ")" << std::endl;
				cx += pt.x;
				cy += pt.y;
			}

			cx /= victim.size();
			cy /= victim.size();
			Point center(cx, cy);

			// Ids of victims are put here with their index i in the sorted_index
			// so that after sorting them we can get the center just with this index
			sorted_index.emplace_back(id, i);
			victim_centers.emplace_back(center);

			std::cout << i << "th victim id: " << id << " - Center: (" << center.x << "," << center.y << ")" << std::endl;
		}

		// Debug prints
		std::cout << "Unsorted list of ids-index:" << std::endl;
		for (int i = 0; i < sorted_index.size(); ++i){
			std::cout << "\t  Id - index: " << sorted_index[i].first << " - " << sorted_index[i].second << std::endl;
		}

		sort(sorted_index.begin(), sorted_index.end());

		// Debug prints
		std::cout << "Sorted list of ids-index:" << std::endl;
		for (int i = 0; i < sorted_index.size(); ++i){
			std::cout << "\tS Id - index: " << sorted_index[i].first << " - " << sorted_index[i].second << std::endl;
		}

		// Final check: now we can get victims in order
		for (int i = 0; i < sorted_index.size(); ++i){
			int id = sorted_index[i].first;
			int index = sorted_index[i].second;
			Point center = victim_centers[index];
			point_list.emplace_back(center);
			std::cout << i << "th victim id: " << id << " - Center: (" << center.x << "," << center.y << ")" << std::endl;
		}

		// Centroid of gate
		double cx = 0, cy = 0;
		for (const auto& pt: gate){

		//	std::cout << "\tPoint: (" << pt.x << "," << pt.y << ")" << std::endl;
			cx += pt.x;
			cy += pt.y;
		}

		cx /= gate.size();
		cy /= gate.size();
		Point gate_center(cx, cy);
		point_list.emplace_back(gate_center);

		// Viualize
		if (DEBUG){
			int radiusCircle = 2;
			cv::Scalar colorCircle1(0,0,255);
			int thicknessCircle1 = 2;

			// Debug: draw victim centers
			for (const Point& pt : point_list) {
				cv::Point visualCent(pt.x*100, pt.y*100);
				cv::circle(image, visualCent, radiusCircle, colorCircle1, thicknessCircle1);				
			}
		
		//	cv::Point visualCent(gate_center.x*100, gate_center.y*100);
		//	cv::circle(image, visualCent, radiusCircle, colorCircle1, thicknessCircle1);
	        char centers[] = "Victim and gate centers";
			cv::namedWindow(centers, 10);
			cv::imshow(centers, image);
			cv::waitKey(0); 
		}


		// FIND DUBINS PATH TO CONNECT THE POINTS
		// final configuration is given by the gate center and an angle that should be in a certain range:
		// theta = 1.45 - 1.65 if the gate is above, 4.6 - 4.8 if it is below
		configuration robot;
		robot.x = x;
		robot.y = y;
		robot.th = theta;

		std::vector<dubinsCurve> curves = multipoint(robot, point_list);

		for (const auto& curve: curves){

			path.points.emplace_back(curve.a1.len, curve.a1.currentConf.x, curve.a1.currentConf.y,curve.a1.currentConf.th, curve.a1.k);

			path.points.emplace_back(curve.a2.len, curve.a2.currentConf.x, curve.a2.currentConf.y,curve.a2.currentConf.th, curve.a2.k);

			path.points.emplace_back(curve.a3.len, curve.a3.currentConf.x, curve.a3.currentConf.y,curve.a3.currentConf.th, curve.a3.k);
		}

	    //float xc = 0, yc = 1.5, r = 1.4;
	    //float ds = 0.05;
	    //for (float theta = -M_PI/2, s = 0; theta<(-M_PI/2 + 1.2); theta+=ds/r, s+=ds) {
	    //  path.points.emplace_back(s, xc+r*std::cos(theta), yc+r*std::sin(theta), theta+M_PI/2, 1./r);    
	    //}

	    return true;
  }
}