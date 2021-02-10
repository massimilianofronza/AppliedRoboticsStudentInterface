#include "path_functions.hpp"

namespace student{

	Polygon this_borders;
	polygon_type arena, valid_gate;
	std::vector<polygon_type> this_obstacle_list;
	std::vector<Polygon> coll_obstacles;
	std::vector<Point> full_tree;
	double SOL_TIME = 1.0;
	cv::Mat graph_image;
	bool done = false;

	/// Adjacency list of sampled points, with 
	/// - index: it is the index of the node in the full tree
	/// - int: index of next node 
	/// - double: cost of link in euclidea distance
 	std::vector<std::vector<triplet>> free_edges;

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
				double s1_x = s1->as<ob::SE2StateSpace::StateType>()->getX();
				double s1_y = s1->as<ob::SE2StateSpace::StateType>()->getY();
				double s2_x = s2->as<ob::SE2StateSpace::StateType>()->getX();
				double s2_y = s2->as<ob::SE2StateSpace::StateType>()->getY();

				if (myStateValidityCheckerFunction(s1) && myStateValidityCheckerFunction(s2)) {
					
					for (const auto& obstacle : coll_obstacles) {
						
						for(int i=1; i<obstacle.size(); i++) {
							//std::cout << "i: " << i << " and " << obstacle.size() << std::endl;
							double start_x = obstacle[i-1].x;
							double start_y = obstacle[i-1].y;
							double end_x = obstacle[i].x;
							double end_y = obstacle[i].y;
							
							if (coll_LineLine(s1_x, s1_y, s2_x, s2_y, start_x, start_y, end_x, end_y)) {
								return false;	/// Collision detected
							}

						}
						/// Close with the last segment
						if (coll_LineLine(s1_x, s1_y, s2_x, s2_y, 
										  obstacle[0].x, obstacle[0].y, obstacle[obstacle.size()-1].x, obstacle[obstacle.size()-1].y)) {
							return false;	/// Collision detected
						}
						
					}

					if (!done){
							cv::Point p1(s1_x*400, 400-s1_y*400);
							cv::Point p2(s2_x*400, 400-s2_y*400);
							cv::line(graph_image, p1, p2, cv::Scalar(255,0,0), 1, cv::LINE_8);	
					  		
					  		full_tree.push_back(Point(s1_x,s1_y));

					  		full_tree.push_back(Point(s2_x,s2_y));
					  		
					  		Point n1(s1_x,s1_y), n2(s2_x,s2_y);
							double dist = distance(n1,n2);
							int index1 = full_tree.size()-2;
							int index2 = full_tree.size()-1;

							free_edges.emplace_back({index1, index2, dist});

					}

					return true;
				}
				else {
					return false;
				}
			}

			bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const override {
				cv::Mat image = cv::Mat::zeros(800, 800, CV_8UC3);		// TODO delete this
				char name[] = "I'M THE SECOND checkMotion()";
				cv::namedWindow(name, 10);
				cv::imshow(name, image);
				cv::waitKey(0);
				cv::destroyWindow(name); 		
				return true;
			}
	};

	bool myStateValidityCheckerFunction(const ob::State *state) {
  		//ob::SE2StateSpace::StateType *D2state = state->as<ob::SE2StateSpace::StateType>();
  		double x = state->as<ob::SE2StateSpace::StateType>()->getX(); 
  		double y = state->as<ob::SE2StateSpace::StateType>()->getY();
  		
  		//std::cout << "Checking validity of point " << x << "," << y << std::endl;

  		point_type p(x, y);
  		if (!(bg::within(p, arena))) {	/// Point external w.r.t. the arena
  			return false;
  		}

  		if (bg::within(p, valid_gate)){
  			return true;
  		}

  		for (const auto& obstacle : this_obstacle_list) {
  			if (bg::within(p, obstacle)) {	/// Point inside some obstacle
	  			return false;
  			}
  		}

  		/// Check if points is already there
/*  		for (int i=0; i<full_tree.size(); i++) {
  			if ((x == full_tree[i].x) && (y == full_tree[i].y)) {
  				std::cout << "Point already thereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee\n";
  			}
  			else {
  				full_tree.push_back(Point(x, y));
  			}
  		}
*/

		return true;
	}


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

		graph_image = cv::Mat::zeros(600, 900, CV_8UC3);
		graph_image.setTo(cv::Scalar(255, 255, 255));

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

			Point center = getCenter(victim);

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


		double gate_mid_w, gate_mid_h;
  		double gate_th = gate_angle(gate, borders, gate_mid_w, gate_mid_h);
  		std::cout << "BIG GATE ANGLE: " << gate_th << std::endl;

		Point gate_center = getCenter(gate);
		if (gate_th == 0){
			gate_center.x -= gate_mid_w;
		} 
		else if (gate_th == PI){
			gate_center.x += gate_mid_w;
		}
		else if (gate_th == PI/2.0){			
			gate_center.y -= gate_mid_h;
		}
		else if (gate_th == 3.0*PI/2.0){			
			gate_center.y += gate_mid_h;
		}

		point_list.emplace_back(gate_center);

		// Viualize
		if (DEBUG_plan){
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

		// Setup planner
		auto space(std::make_shared<ob::SE2StateSpace>());
    	
		// set the bounds for the R^2 part of SE(2)
	    ob::RealVectorBounds bounds(2);
	    bounds.setLow(0);
	    bounds.setHigh(1.6);
	  
	  	space->setBounds(bounds);	

		// Construct a space information instance for this state space
		auto si(std::make_shared<ob::SpaceInformation>(space));

		// Set the object used to check which states in the space are valid
		si->setStateValidityChecker(myStateValidityCheckerFunction);
		si->setStateValidityCheckingResolution(0.03);
		si->setValidStateSamplerAllocator(allocOBValidStateSampler);

		//si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
		si->setMotionValidator(std::make_shared<myMotionValidator>(si));
		//ob::MotionValidatorPtr mvPtr(new myMotionValidator(si));//ob::DiscreteMotionValidator(si));
		//si->setMotionValidator(mvPtr);
		si->setup();
		
		std::vector<Point> RRT_list;

		
  		missionOne(point_list, RRT_list, si, space, image);
  		missionTwo(si, point_list);
  		

		cv::Mat sol_image = cv::Mat::zeros(600, 600, CV_8UC3);
		sol_image.setTo(cv::Scalar(255, 255, 255));
		drawSolutionTree(RRT_list, sol_image);

		// Try to connect Gate directly to last victim
/*		point_type final_p(RRT_list[RRT_list.size()-1].x, RRT_list[RRT_list.size()-1].y);

		if (!bg::within(final_p, valid_gate)){	/// Final point not on the gate
  			if ((point_list[point_list.size()-2].x == RRT_list[RRT_list.size()-1].x) &&
  				(point_list[point_list.size()-2].y == RRT_list[RRT_list.size()-1].y)) {		/// Final victim exactly the last-1 point found
  				/// TODO collision checking
  				RRT_list.push_back(point_list[point_list.size()-1]);						/// Append exact gate
  			}
  			else {
  				RRT_list[RRT_list.size()-1] = point_list[point_list.size()-1];
  			}
  		}
*/

  		// FIND DUBINS PATH TO CONNECT THE POINTS
		// final configuration is given by the gate center and an angle that should be in a certain range:
		// theta = 1.45 - 1.65 if the gate is above, 4.6 - 4.8 if it is below
		configuration robot;
		robot.x = x;
		robot.y = y;
		robot.th = theta;

		std::vector<dubinsCurve> curves = multipoint(robot, RRT_list, gate_th);

		float ds = 0.001;
		bool entered = false;
		int counter = 0;

		for (const auto& curve: curves){

			double trace = 0;

			for (float s = 0; s < curve.a1.len; s+=ds) {
				if (!entered) {
					std::cout << "\tEntering 1/3\n";
					entered = true;
				}
				configuration intermediate = getNextConfig(curve.a1.currentConf, curve.a1.k, s);
				path.points.emplace_back(s, intermediate.x, intermediate.y, intermediate.th, curve.a1.k);
				//if ((curve.a1.len-s) < ds) {
				//	s += curve.a1.len-s-ds;
				//}
				trace = s;
			}

			entered = false;
			//std::cout << "MOVING, expected a1: " << curve.a1.len << ", performed: " << (trace-curve.a1.len) << ", trace: " << trace << std::endl;
			float old_trace = trace;

			for (float s = old_trace; s < curve.a2.len + old_trace; s+=ds) {
				if (!entered) {
					std::cout << "\tEntering 2/3\n";
					entered = true;
				}
				configuration intermediate = getNextConfig(curve.a2.currentConf, curve.a2.k, s-old_trace);
				path.points.emplace_back(s, intermediate.x, intermediate.y, intermediate.th, curve.a2.k);
				//if ((curve.a2.len-s) < ds) {
				//	s += curve.a2.len-s-ds;
				//}
				trace = s;
			}

			entered = false;
			//std::cout << "MOVING, expected a2: " << curve.a2.len << ", performed: " << (trace-curve.a2.len) << ", trace: " << trace << std::endl;
			old_trace = trace;

			for (float s = old_trace; s < curve.a3.len + old_trace; s+=ds) {
				if (!entered) {
					std::cout << "\tEntering 3/3\n";
					entered = true;
				}
				configuration intermediate = getNextConfig(curve.a3.currentConf, curve.a3.k, s-old_trace);
				path.points.emplace_back(s, intermediate.x, intermediate.y, intermediate.th, curve.a3.k);
				//if ((curve.a3.len-s) < ds) {
				//	s += curve.a3.len-s-ds;
				//}
				trace = s;
			}

			entered = false;
			//std::cout << "MOVING, expected a3: " << curve.a3.len << ", performed: " << (trace-curve.a3.len) << ", trace: " << trace << std::endl;
			std::cout << "Expected curve length: " << curve.L << ", performed: " << trace << std::endl;
			counter++;

		}

		std::cout << "Total # of curves: " << counter << std::endl;

	    return true;
  	}


  	void missionOne(std::vector<Point>& point_list, std::vector<Point>& RRT_list, std::shared_ptr<ompl::base::SpaceInformation> si, std::shared_ptr<ob::SE2StateSpace> space, cv::Mat& image){
  		for (int i=1; i < point_list.size(); i++) {
  			if(i==2){
  				done = true;
  				char graph[] = "Graph";
				cv::namedWindow(graph, 10);
				cv::imshow(graph, graph_image);
				cv::waitKey(0);
				cv::destroyWindow(graph); 
  			}
			// Create an instance of a planner
			auto planner(std::make_shared<og::RRTstar>(si));

			// Set our robot's starting state to be the bottom-left corner of
			// the environment, or (0,0).
			ob::ScopedState<> start(space);
			start->as<ob::SE2StateSpace::StateType>()->setX(point_list[i-1].x);
			start->as<ob::SE2StateSpace::StateType>()->setY(point_list[i-1].y);
		

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
			ob::PlannerStatus solved = planner->ob::Planner::solve(SOL_TIME);

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

		        if (DEBUG_plan) {
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

			pdef->clearGoal();	/// Free the memory from the previous goal TODO remove
		}
  	}


  	void missionTwo(std::shared_ptr<ompl::base::SpaceInformation> si, std::vector<Point> &point_list) {
  		
  	/*	
	    // graph edges array.
	    graphEdge edges[] = {
	        // (x, y, w) -> edge from x to y with weight w
	        {0,1,2}, {0,2,4}, {1,4,3}, {2,3,2}, {3,1,4}, {4,3,3}
	    };
	    int N = 6;      // Number of vertices in the graph
	    // calculate number of edges
	    int n = sizeof(edges)/sizeof(edges[0]);

	    // construct graph
	    graph adjGraph(free_edges, n, free_edges.size());


	    // print adjacency list representation of graph
	    std::cout << "Graph adjacency list " << std::endl << "(startNode, endNode, weight): " << std::endl;
	    for (int i = 0; i < N; i++) {
	        // display adjacent vertices of vertex i
	        printAdjList(adjGraph.head[i], i);
	    }

  		for (int i=1; i<point_list.size(); i++) {

  		}
	*/
  	}


}