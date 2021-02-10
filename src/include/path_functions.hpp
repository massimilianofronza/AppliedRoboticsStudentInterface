#include "utils.hpp"
#include "collision_functions.hpp"
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "visual_functions.hpp"

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
#include <algorithm>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bg = boost::geometry;
/// End of planning imports

namespace student {

	const bool DEBUG_plan = false;

	struct adjNode {
		int id, cost;
		adjNode* next;
	};

	// structure to store edges
	struct graphEdge {
	    int startNode, endNode, weight;
	};
	
	class graph {
	    // insert new nodes into adjacency list from given graph
	    adjNode* getAdjListNode(int newId, int weight, adjNode* head) {
	        adjNode* newNode = new adjNode;
	        newNode->id = newId;
	        newNode->cost = weight; 
	        newNode->next = head;   // point new node to current head
	        return newNode;
	    }
	    
	    int N;  // number of nodes in the graph
	
	public:
	    adjNode **head;                //adjacency list as array of pointers
	    // Constructor
	    graph(graphEdge edges[], int n, int N) {
	        
	        // allocate new node
	        head = new adjNode*[N]();
	        this->N = N;
	        
	        // initialize head pointer for all vertices
	        for (int i = 0; i < N; i++) {
	            head[i] = nullptr;
	        }

	        // construct directed graph by adding edges to it
	        for (unsigned i = 0; i < n; i++)  {
	            int startNode = edges[i].startNode;
	            int endNode = edges[i].endNode;
	            int weight = edges[i].weight;
	            
	            // insert in the beginning
	            adjNode* newNode = getAdjListNode(endNode, weight, head[startNode]);
	             
				// point head pointer to new node
				head[startNode] = newNode;
			}
	    }

	    // Destructor
	    ~graph() {
		    for (int i = 0; i < N; i++) {
		        delete[] head[i];
		    }
		    delete[] head;
	    }
	};

	void printAdjList(adjNode* ptr, int i);


	Point getCenter(const Polygon &poly);
	bool myStateValidityCheckerFunction(const ob::State *state);
	ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation* si);
	void drawSolutionTree(std::vector<Point> RRT_list, cv::Mat& image);
	double gate_angle(const Polygon &gate, const Polygon &arena, double& gate_mid_w, double& gate_mid_h);

	typedef bg::model::d2::point_xy<double> point_type;
	typedef bg::model::polygon<point_type> polygon_type;

	void missionTwo(std::shared_ptr<ompl::base::SpaceInformation> si, std::vector<Point> &point_list);
  	
  	void missionOne(std::vector<Point>& point_list, std::vector<Point>& RRT_list, std::shared_ptr<ompl::base::SpaceInformation> si, std::shared_ptr<ob::SE2StateSpace> space, cv::Mat& image);

  	int find_closest(Point &self, std::vector<Point> &samples);
	double distance(Point &a, Point &b);
  	
	////////// PATH FUNCTIONS CALLED BY student_interface.cpp //////////

	//void student_loadImage(cv::Mat& img_out, const std::string& config_folder);
	bool student_planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
              const std::vector<std::pair<int,Polygon>>& victim_list, 
              const Polygon& gate, const float x, const float y, const float theta, 
              Path& path,
              const std::string& config_folder);
}
