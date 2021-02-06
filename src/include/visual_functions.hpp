#include <opencv2/opencv.hpp>
#include "utils.hpp"

namespace student {

	// GLOBAL VARIABLES	
	extern double SCALE;
	
	////////// VISUAL FUNCTIONS CALLED BY student_interface.cpp //////////

	void student_loadImage(cv::Mat& img_out, const std::string& config_folder);
	
	void student_genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder);
	
	void student_imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
				const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, 
				const std::string& config_folder);

	bool student_extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points,
				const cv::Mat& camera_matrix, cv::Mat& rvec,
				cv::Mat& tvec, const std::string& config_folder);

	void student_findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec,
				const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane,
				const std::vector<cv::Point2f>& dest_image_points_plane, cv::Mat& plane_transf,
				const std::string& config_folder);

	void student_unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf,
				const std::string& config_folder);

	bool student_findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, 
				double& x, double& y, double& theta, const std::string& config_folder, const bool DEBUG);

	bool student_processMap(const cv::Mat& img_in, const double scale,
				std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list,
				Polygon& gate, const std::string& config_folder, const bool DEBUG);

}
