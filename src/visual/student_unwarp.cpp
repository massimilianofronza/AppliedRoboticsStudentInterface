#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "visual_functions.hpp"

namespace student {
	void student_unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
            const std::string& config_folder) {

		// Debug variable
		static bool initialized = false;

		try {
			cv::warpPerspective(img_in, img_out, transf, img_in.size());
		}
		catch (const cv::Exception& e) {
			std::cerr << "ERROR IN METHOD <warpPerspective> of student_unwarp.cpp: " << e.msg << std::endl;
			return;
		}

		if (!initialized) {
			std::cout << "\tstudent_unwarp EXECUTED.\n" << std::flush;
			initialized = true;
		}
	}
}
