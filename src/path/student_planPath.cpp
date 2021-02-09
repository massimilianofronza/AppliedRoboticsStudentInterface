#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "visual_functions.hpp"
#include "path_functions.hpp"
#include "collision_functions.hpp"

/// Path planning specific imports
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/geometric/PathGeometric.h>
  
#include <ompl/config.h>
#include <iostream>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bg = boost::geometry;
/// End of planning imports

namespace student{

	const bool DEBUG = true;

	bool myStateValidityCheckerFunction(const ob::State *state);
	ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation*si);
		void drawSolutionTree(std::vector<Point> RRT_list, cv::Mat& image);
	typedef bg::model::d2::point_xy<double> point_type;
	typedef bg::model::polygon<point_type> polygon_type;
	
	Polygon this_borders;
	polygon_type arena, valid_gate;
	std::vector<polygon_type> this_obstacle_list;
	std::vector<Polygon> coll_obstacles;
	std::vector<Point> full_tree;

	/// Class for motion validation
	class myMotionValidator : public ob::MotionValidator {
		
		private:
			ob::StateSpace *stateSpace_;

		public:

			myMotionValidator(ob::SpaceInformation *si) : ob::MotionValidator(si) {
				//ob::MotionValidator::defaultSettings();
				stateSpace_ = si_->getStateSpace().get();
	    		if (stateSpace_ == nullptr) {
	        		throw ompl::Exception("No state space for motion validator");
	    		}
			}

			myMotionValidator(const ob::SpaceInformationPtr &si) : ob::MotionValidator(si) {
				stateSpace_ = si_->getStateSpace().get();
	    		if (stateSpace_ == nullptr) {
	        		throw ompl::Exception("No state space for motion validator");
	    		}
			}

			bool checkMotion(const ob::State *s1, const ob::State *s2) const override {
				//std::cout << "CIAO FRA STO USANDO LA PRIMA\n";
				//full_tree.push_back(Point(s1->as<ob::SE2StateSpace::StateType>()->getX(), 
				//						  s1->as<ob::SE2StateSpace::StateType>()->getY()));
				double s1_x = s1->as<ob::SE2StateSpace::StateType>()->getX();
				double s1_y = s1->as<ob::SE2StateSpace::StateType>()->getY();
				double s2_x = s2->as<ob::SE2StateSpace::StateType>()->getX();
				double s2_y = s2->as<ob::SE2StateSpace::StateType>()->getY();

				if (myStateValidityCheckerFunction(s1) && myStateValidityCheckerFunction(s2)) {
					
					for (const auto& obstacle : coll_obstacles) {
						
						for(int i=1; i<obstacle.size(); i++) {
							std::cout << "i: " << i << " and " << obstacle.size() << std::endl;
							double start_x = obstacle[i-1].x;
							double start_y = obstacle[i-1].y;
							double end_x = obstacle[i].x;
							double end_y = obstacle[i].y;
							
							if (coll_LineLine(s1_x, s1_y, s2_x, s2_y, start_x, start_y, end_x, end_y)) {
								std::cerr << "Collision detected for new sample point.\n";
								return false;
							}
						}
						/// Close with the last segment
						if (coll_LineLine(s1_x, s1_y, s2_x, s2_y, 
										  obstacle[0].x, obstacle[0].y, obstacle[obstacle.size()-1].x, obstacle[obstacle.size()-1].y)) {
							std::cerr << "Collision detected for new sample point.\n";
							return false;
						}
					}
				}

				return true;
			}

			bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const override {
				std::cout << "CIAO FRA STO USANDO LA SECONDA\n";
				return true;
			}
	};

	// TODO represent arena and obstacles with the boost geometry types to check
	// if the points are inside the polygons (for state validity)
	// check: boost.org/doc/libs/1_62_0/libs/geometry/doc/html/geometry/reference/algorithms/within/within_2.html
	
	/**
	* Performs the computation of a collision free path, that goes through all the victims's centers 
	* in order (mission 1) by using Dubin's manoeuvres.
	*
	*/
	bool student_planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
                 const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
                 const float x, const float y, const float theta, Path& path,
                 const std::string& config_folder){

		this_borders = borders;
		coll_obstacles = obstacle_list;

		for (const auto& vertex : borders){
			std::cout << "Arena corner: " << vertex.x << "," << vertex.y << std::endl;
			bg::append(arena.outer(), point_type(vertex.x, vertex.y));
		}
		/// Close the arena polygon
		bg::append(arena.outer(), point_type(borders[0].x, borders[0].y));

		for (const auto& obstacle : obstacle_list) {
			polygon_type polygon;

			for (const auto& obst_vertex : obstacle) {
				bg::append(polygon.outer(), point_type(obst_vertex.x, obst_vertex.y));
			}
			/// Close the obstacle polygon
			bg::append(polygon.outer(), point_type(obstacle[0].x, obstacle[0].y));

			this_obstacle_list.emplace_back(polygon);
		}

		for (const auto& vertex : gate){
			std::cout << "Gate corner: " << vertex.x << "," << vertex.y << std::endl;
			bg::append(valid_gate.outer(), point_type(vertex.x, vertex.y));
		}

		// TODO these two global variables don't work
		//std::cout << "Inside student_planPath, size of offsetted_obstacles: " << offsetted_obstacles.size() << std::endl; // here, should be != 0
		//std::cout << "SCALE " << SCALE << std::endl;
		// debug image
		cv::Mat image = cv::Mat::zeros(600, 600, CV_8UC3);

		image.setTo(cv::Scalar(255, 255, 255));


		// Variables to keep victims and their centroid sorted 
		std::vector<std::pair<int, int>> sorted_index;
		std::vector<Point> victim_centers;

		// A list of sorted points to be visited, including robot and gate (to pass to Dubins)
		std::vector<Point> point_list;

		point_list.emplace_back(x,y);

		// Process victims, compute their centroid and populate the above variables
		for (int i = 0; i < victim_list.size(); i++){
			int id = victim_list[i].first;
			Polygon victim = victim_list[i].second;

			//std::cout << i << "^ victim id: " << id << std::endl;

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

			std::cout << i << "^ victim id: " << id << " - Center: (" << center.x << "," << center.y << ")" << std::endl;
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
			std::cout << i << "^ victim id: " << id << " - Center: (" << center.x << "," << center.y << ")" << std::endl;
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
				cv::Point visualCent(pt.x*300, pt.y*300);
				cv::circle(image, visualCent, radiusCircle, colorCircle1, thicknessCircle1);				
			}
		
		//	cv::Point visualCent(gate_center.x*100, gate_center.y*100);
		//	cv::circle(image, visualCent, radiusCircle, colorCircle1, thicknessCircle1);
	        char centers[] = "Victim and gate centers";
			cv::namedWindow(centers, 10);
			cv::imshow(centers, image);
			cv::waitKey(0);
			cv::destroyWindow(centers); 
		}


		// FIND DUBINS PATH TO CONNECT THE POINTS
		// final configuration is given by the gate center and an angle that should be in a certain range:
		// theta = 1.45 - 1.65 if the gate is above, 4.6 - 4.8 if it is below
		configuration robot;
		robot.x = x;
		robot.y = y;
		robot.th = theta;

		

	/*	//CODE from the professor interface
	    float xc = 0, yc = 1.5, r = 1.4;
	    float ds = 0.05;
	    std::cout << "M_PI =" << M_PI << std::endl;
	    for (float theta = -M_PI/2, s = 0; theta<(-M_PI/2 + 1.2); theta+=ds/r, s+=ds) {
	    	
	      path.points.emplace_back(s, xc+r*std::cos(theta), yc+r*std::sin(theta), theta+M_PI/2, 1./r);    
	    }
*/

		auto space(std::make_shared<ob::SE2StateSpace>());
		//ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    	
		// set the bounds for the R^2 part of SE(2)
	    ob::RealVectorBounds bounds(2);
	    bounds.setLow(0);
	    bounds.setHigh(1.6);
	  
	  	space->setBounds(bounds);


    	// Set the bounds of space to be in [0,1].
		//space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 2.0);		

		// Construct a space information instance for this state space
		auto si(std::make_shared<ob::SpaceInformation>(space));

		// Set the object used to check which states in the space are valid
		si->setStateValidityChecker(myStateValidityCheckerFunction);
		si->setStateValidityCheckingResolution(0.01);
		si->setValidStateSamplerAllocator(allocOBValidStateSampler);

		si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
		//ob::MotionValidatorPtr mvPtr(new ob::DiscreteMotionValidator(si));
		//si->setMotionValidator(mvPtr);

		si->setup();
		
		std::vector<Point> RRT_list;

		for (int i=1; i < point_list.size(); i++) {
			// Create an instance of a planner
			auto planner(std::make_shared<og::RRTstar>(si));

			// Set our robot's starting state to be the bottom-left corner of
			// the environment, or (0,0).
			ob::ScopedState<> start(space);
			start->as<ob::SE2StateSpace::StateType>()->setX(point_list[i-1].x);
			start->as<ob::SE2StateSpace::StateType>()->setY(point_list[i-1].y);
			//start->as<ob::SE2StateSpace::StateType>()->setYaw(robot.th);

			ob::ScopedState<> goal(space);
			goal->as<ob::SE2StateSpace::StateType>()->setX(point_list[i].x);
			goal->as<ob::SE2StateSpace::StateType>()->setY(point_list[i].y);

			std::cout << "i: " << i << ", start: " << start << ", goal: " << goal << std::endl;

			auto pdef(std::make_shared<ob::ProblemDefinition>(si));

			// Set the start and goal states
			pdef->setStartAndGoalStates(start, goal);

			// Tell the planner which problem we are interested in solving
			planner->setProblemDefinition(pdef);
			planner->setup();

			// Returns a value from ompl::base::PlannerStatus which describes whether a solution has been found within the specified 
			// amount of time (in seconds). If this value can be cast to true, a solution was found.
			ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);

			// If a solution has been found, display it. Simplification could be done, but we would need to create an instance 
			// of ompl::geometric::PathSimplifier.
			if (solved) {
				// get the goal representation from the problem definition (not the same as the goal state)
			    // and inquire about the found path
				og::PathGeometric path(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
				const std::vector<ob::State*> &states = path.getStates();
				ob::SE2StateSpace::StateType *state;

				for(size_t j=0; j < states.size() ; ++j) {
					if (i==1) {
						state = states[j]->as<ob::SE2StateSpace::StateType>();
						RRT_list.emplace_back(state->getX(), state->getY());
						std::cout << "State " << j << ": " << state->getX() << ", " << state->getY() << "\n";
					}
					else {
						if (j!=0) {
							state = states[j]->as<ob::SE2StateSpace::StateType>();
							RRT_list.emplace_back(state->getX(), state->getY());
							std::cout << "State " << j << ": " << state->getX() << ", " << state->getY() << "\n";
						}
					}
				}

				for (int k=1; k<RRT_list.size(); k++) {
					double s1_x = RRT_list[k-1].x;
					double s1_y = RRT_list[k-1].y;
					double s2_x = RRT_list[k].x;
					double s2_y = RRT_list[k].y;

					for (const auto& obstacle : coll_obstacles) {

						for(int l=1; l<obstacle.size(); l++) {
							double start_x = obstacle[l-1].x;
							double start_y = obstacle[l-1].y;
							double end_x = obstacle[l].x;
							double end_y = obstacle[l].y;
							
						//	if (coll_LineLine(s1_x, s1_y, s2_x, s2_y, start_x, start_y, end_x, end_y)) {
						//		std::cerr << "Collision detected for final path.\n";
								//return false;
						//	}
						}
						/// Close with the last segment
		/*				if (coll_LineLine(s1_x, s1_y, s2_x, s2_y, 
										  obstacle[0].x, obstacle[0].y, obstacle[obstacle.size()-1].x, obstacle[obstacle.size()-1].y)) {
							std::cerr << "Collision detected for final path.\n";
							//return false;
						}
		*/
					}
				}

		        //og::PathGeometric path = pdef->as<og::PathGeometric>getSolutionPath();
		        std::cout << "Found solution: " << i << std::endl;
		 
		        // print the path to screen
		        path.print(std::cout);

		        if (DEBUG){
					int radiusCircle = 1;
					cv::Scalar colorCircle1(255,0,0);
					int thicknessCircle1 = 2;

					// Debug: draw victim centers
					for (const Point& pt : full_tree) {
						cv::Point visualCent(pt.x*300, pt.y*300);
						cv::circle(image, visualCent, radiusCircle, colorCircle1, thicknessCircle1);				
					}
				
			        char centers[] = "Full points on arena";
					cv::namedWindow(centers, 10);
					cv::imshow(centers, image);
					cv::waitKey(0);
					cv::destroyWindow(centers); 
				}
			}
		}

		cv::Mat sol_image = cv::Mat::zeros(600, 600, CV_8UC3);
		sol_image.setTo(cv::Scalar(255, 255, 255));
		drawSolutionTree(RRT_list, sol_image);

		std::vector<dubinsCurve> curves = multipoint(robot, RRT_list);

		float ds = 0.02;

		for (const auto& curve: curves){

			for (float s = 0; s <= curve.a1.len; s+=ds) {
				configuration intermediate = getNextConfig(curve.a1.currentConf, curve.a1.k, s);
				path.points.emplace_back(s, intermediate.x, intermediate.y, intermediate.th, curve.a1.k);
			}

			for (float s = 0; s <= curve.a2.len; s+=ds) {
				configuration intermediate = getNextConfig(curve.a2.currentConf, curve.a2.k, s);
				path.points.emplace_back(s, intermediate.x, intermediate.y, intermediate.th, curve.a2.k);
			}

			for (float s = 0; s <= curve.a3.len; s+=ds) {
				configuration intermediate = getNextConfig(curve.a3.currentConf, curve.a3.k, s);
				path.points.emplace_back(s, intermediate.x, intermediate.y, intermediate.th, curve.a3.k);
			}
			//path.points.emplace_back(curve.a1.len, curve.a1.currentConf.x, curve.a1.currentConf.y,curve.a1.currentConf.th, curve.a1.k);
		
			//path.points.emplace_back(curve.a2.len, curve.a2.currentConf.x, curve.a2.currentConf.y,curve.a2.currentConf.th, curve.a2.k);
		
			//path.points.emplace_back(curve.a3.len, curve.a3.currentConf.x, curve.a3.currentConf.y,curve.a3.currentConf.th, curve.a3.k);
		}


	    return true;
  	}

  	
  	bool myStateValidityCheckerFunction(const ob::State *state) {
  		//ob::SE2StateSpace::StateType *D2state = state->as<ob::SE2StateSpace::StateType>();
  		double x = state->as<ob::SE2StateSpace::StateType>()->getX(); 
  		double y = state->as<ob::SE2StateSpace::StateType>()->getY();
  		
  		//std::cout << "Checking validity of point " << x << "," << y << std::endl;

  		point_type p(x, y);
  		if (!(bg::within(p, arena))) {
  			//std::cerr << "Point external w.r.t. the arena.\n";
  			return false;
  		}

  		if (bg::within(p, valid_gate)){
  			return true;
  		}

  		for (const auto& obstacle : this_obstacle_list) {
  			if (bg::within(p, obstacle)) {
	  			//std::cerr << "Point inside some obstacle.\n";
	  			return false;
  			}
  		}

		full_tree.push_back(Point(x, y));
  		
		return true;
	}

	ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation*si){
		return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
	}

	void drawSolutionTree(std::vector<Point> RRT_list, cv::Mat& image){
		for(int i = 1; i< RRT_list.size(); i++){
			cv::Point p1(RRT_list[i-1].x*300, RRT_list[i-1].y*300);
			cv::Point p2(RRT_list[i].x*300, RRT_list[i].y*300);
			cv::line(image, p1, p2, cv::Scalar(255,0,0), 3, cv::LINE_8);
		}
		char tree[] = "Tree";
		cv::namedWindow(tree, 10);
		cv::imshow(tree, image);
		cv::waitKey(0);
		cv::destroyWindow(tree); 

	}

}