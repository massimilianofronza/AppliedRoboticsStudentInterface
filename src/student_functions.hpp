#include <opencv2/opencv.hpp>
//#include "utils.hpp"

namespace student {

//	FUNCTIONS CALLED BY student_interface.cpp WILL BE DEFINED HERE

	void student_imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
				    const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, 
                    const std::string& config_folder);

	bool student_extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, 				cv::Mat& tvec, 	const std::string& config_folder);

	void student_findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& 			object_points_plane, const std::vector<cv::Point2f>& dest_image_points_plane, cv::Mat& plane_transf, const std::string& config_folder);
    		 
  
}