#include "path_functions.hpp"

namespace student {

	/**
	 * Returns the Point struct with the coordinates of the passed Polygon's center
	 */
	Point getCenter(const Polygon &poly) {
		
		double cx = 0, cy = 0;

		for (const auto& pt: poly) {
		//	std::cout << "\tPoint: (" << pt.x << "," << pt.y << ")" << std::endl;
			cx += pt.x;
			cy += pt.y;
		}

		cx /= poly.size();
		cy /= poly.size();
		Point center(cx, cy);

		return center;
	}

	ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation*si){
		return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
	}

	void drawSolutionTree(std::vector<Point> RRT_list, cv::Mat& image){
		for(int i = 1; i< RRT_list.size(); i++){
			cv::Point p1(RRT_list[i-1].x*300, 500-RRT_list[i-1].y*300);
			cv::Point p2(RRT_list[i].x*300, 500-RRT_list[i].y*300);
			cv::line(image, p1, p2, cv::Scalar(255,0,0), 3, cv::LINE_8);
		}
		char tree[] = "Tree";
		cv::namedWindow(tree, 10);
		cv::imshow(tree, image);
		cv::waitKey(0);
		cv::destroyWindow(tree); 
	}

	double gate_angle(const Polygon &gate, const Polygon &arena,double& gate_mid_w, double& gate_mid_h) {
		Point gate_center = getCenter(gate);
		Point arena_center = getCenter(arena);
		
		// Computing gate's height and width
		double gate_width = 0;
		double gate_height = 0;
		
		for (int i=1; i<gate.size(); i++) {
			
			double tmp_w = 0;
			if (gate[i-1].x > gate[i].x) {
				tmp_w = gate[i-1].x - gate[i].x;
			}
			else {
				tmp_w = gate[i].x - gate[i-1].x;
			}
			if (tmp_w > gate_width) {
				gate_width = tmp_w;
			}

			double tmp_h = 0;
			if (gate[i-1].y > gate[i].y) {
				tmp_h = gate[i-1].y - gate[i].y;
			}
			else {
				tmp_h = gate[i].y - gate[i-1].y;
			}
			if (tmp_h > gate_height) {
				gate_height = tmp_h;
			}
		}	

		gate_mid_w = gate_width/2.0;
		gate_mid_h = gate_height/2.0;

		// Now deciding the angle
		if (gate_center.x < arena_center.x) {			// Gate on the left side
							
			if (gate_height > gate_width) {
				return PI;
			}
			else {
				if (gate_center.y > arena_center.y) {	// Gate on the upper side
					return PI/2.0;
				}
				else {
					return (3.0*PI/2.0);
				}
			}
		}
		else {											// Gate on the right side
			if (gate_height > gate_width) {
				return 0;
			}
			else {
				if (gate_center.y > arena_center.y) {	// Gate on the upper side
					return PI/2.0;
				}
				else {
					return (3.0*PI/2.0);
				}
			}
		}
	}

	/// Find the point closest to a given point
    int find_closest(Point &self, std::vector<Point> &samples) {
        double min_dist = 99999999;
        int min_index = -1;
 
        for (int i=0; i<samples.size(); i++) {
            double dist = distance(self, samples[i]);
            if(dist < min_dist) {
                min_dist = dist;
                min_index = i;
			}
        }
        return min_index;
    }

    /// Returns an euclidean distance
    double distance(Point &a, Point &b) {
    	return sqrt(pow((a.x - b.x), 2.0) + pow((a.y - b.y), 2.0));
    }

	// print all adjacent vertices of given vertex
    void printAdjList(adjNode* ptr, int i) {
	    while (ptr != nullptr) {
	        std::cout << "(" << i << ", " << ptr->id
	            << ", " << ptr->cost << ") ";
	        ptr = ptr->next;
	    }
	    std::cout << std::endl;
	}
}
