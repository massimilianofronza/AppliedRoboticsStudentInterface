#include "utils.hpp"
#include <algorithm>

namespace student {

	////////// PATH FUNCTIONS CALLED BY student_interface.cpp //////////

	//void student_loadImage(cv::Mat& img_out, const std::string& config_folder);
	bool student_planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
              const std::vector<std::pair<int,Polygon>>& victim_list, 
              const Polygon& gate, const float x, const float y, const float theta, 
              Path& path,
              const std::string& config_folder);
}
