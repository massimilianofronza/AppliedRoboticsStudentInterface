#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "visual_functions.hpp"
#include "path_functions.hpp"
#include "dubins_functions.hpp"

/// Path planning specific imports
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
  
#include <ompl/config.h>
#include <iostream>
  
namespace ob = ompl::base;
namespace og = ompl::geometric;
/// End of planning imports

namespace student{

	const bool DEBUG = true;

	bool myStateValidityCheckerFunction(const ob::State *state);

	/**
	* Performs the computation of a collision free path, that goes through all the victims's centers 
	* in order (mission 1) by using Dubin's manoeuvres.
	*
	*/
	bool student_planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
                 const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
                 const float x, const float y, const float theta, Path& path,
                 const std::string& config_folder){


		// TODO these two global variables don't work
		//std::cout << "Inside student_planPath, size of offsetted_obstacles: " << offsetted_obstacles.size() << std::endl; // here, should be != 0
		//std::cout << "SCALE " << SCALE << std::endl;
		// debug image
		cv::Mat image = cv::Mat::zeros(300,300, CV_8UC3);

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
				cv::Point visualCent(pt.x*100, pt.y*100);
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
	    bounds.setHigh(2);
	  
	  	space->setBounds(bounds);


    	// Set the bounds of space to be in [0,1].
		//space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 2.0);		

		// Construct a space information instance for this state space
		auto si(std::make_shared<ob::SpaceInformation>(space));

		// Set the object used to check which states in the space are valid
		si->setStateValidityChecker(myStateValidityCheckerFunction);
		si->setStateValidityCheckingResolution(0.03);
		si->setup();
		
		// Set our robot's starting state to be the bottom-left corner of
		// the environment, or (0,0).
		ob::ScopedState<> start(space);
		start->as<ob::SE2StateSpace::StateType>()->setX(robot.x); //values[0] = robot.x;
		start->as<ob::SE2StateSpace::StateType>()->setY(robot.y);
		start->as<ob::SE2StateSpace::StateType>()->setYaw(robot.th);
		 
		// Set our robot's goal state to be the top-right corner of the
		// environment, or (1,1).
		ob::ScopedState<> goal(space);
		goal->as<ob::SE2StateSpace::StateType>()->setX(gate_center.x);
		goal->as<ob::SE2StateSpace::StateType>()->setY(gate_center.y);
		
		auto pdef(std::make_shared<ob::ProblemDefinition>(si));
		
		// Set the start and goal states
		pdef->setStartAndGoalStates(start, goal);

		// Create an instance of a planner
    	auto planner(std::make_shared<og::RRTConnect>(si));

		// Tell the planner which problem we are interested in solving
    	planner->setProblemDefinition(pdef);

		planner->setup();

		// Returns a value from ompl::base::PlannerStatus which describes whether a solution has been found within the specified 
		// amount of time (in seconds). If this value can be cast to true, a solution was found.
    	ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    	std::vector<Point> RRT_list;

		// If a solution has been found, display it. Simplification could be done, but we would need to create an instance 
		// of ompl::geometric::PathSimplifier.
    	if (solved) {
	        // get the goal representation from the problem definition (not the same as the goal state)
	        // and inquire about the found path
	    	og::PathGeometric path( dynamic_cast<const og::PathGeometric&>( *pdef->getSolutionPath()));
			const std::vector< ob::State* > &states = path.getStates();
			ob::SE2StateSpace::StateType *state;

			for(size_t i=0; i < states.size() ; ++i) {
				state = states[i]->as<ob::SE2StateSpace::StateType>();
				RRT_list.emplace_back(state->getX(), state->getY());
				std::cout << "State " << i << ": " << state->getX() << ", " << state->getY() << "\n";
			}
	        //og::PathGeometric path = pdef->as<og::PathGeometric>getSolutionPath();
	        std::cout << "Found solution:" << std::endl;
	 
	        // print the path to screen
	        path.print(std::cout);
    	}
		std::cout << "Start " << start << std::endl;
		std::cout << "Goal " << goal << std::endl;   	


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
		return true;
	}
}