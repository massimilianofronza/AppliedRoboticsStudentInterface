#include <opencv2/opencv.hpp>
//#include "utils.hpp"

namespace student {

//	FUNCTIONS CALLED BY student_interface.cpp WILL BE DEFINED HERE

	void student_imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
				    const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, 
                    const std::string& config_folder);

}