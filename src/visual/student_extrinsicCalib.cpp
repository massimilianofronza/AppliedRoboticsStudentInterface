#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "visual_functions.hpp"

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>


namespace student {

	// Definition of helper functions to pick n points and
	// callback mouseCallback. 
	// The function pickNPoints is used to display a window with a background
	// image, and to prompt the user to select n points on this image.
	// TAKEN FROM professor_interface

  static cv::Mat bg_img;
  static std::vector<cv::Point2f> result;
  static std::string name;
 	static std::atomic<bool> done;
  static int n;
  static double show_scale = 1.0;

 	void mouseCallback(int event, int x, int y, int, void* p) {

   	if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;
    
    result.emplace_back(x*show_scale, y*show_scale);
   	cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
    cv::imshow(name.c_str(), bg_img);

    if (result.size() >= n) {
    	usleep(500*1000);
    	done.store(true);
  	}
  }

  std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img) {
  	result.clear();
  	cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
  	cv::resize(img, bg_img, small_size);
  
  	//bg_img = img.clone();
  	name = "Pick " + std::to_string(n0) + " points";
  	cv::imshow(name.c_str(), bg_img);
  	cv::namedWindow(name.c_str());
  	n = n0;

  	done.store(false);

  	cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
  	while (!done.load()) {
  		cv::waitKey(500);
  	}

 		cv::destroyWindow(name.c_str());
 	 	return result;
 	}

  // Calls the above function to get an estimate of the arena corners in 3D
  // and from there to compute rotation and translation vectors (2D).
  // These will be used to unwarp the image later, transforming it 
  // from 3D to 2D in a correct way.
  // (Finds arena pose from 3D(object_points)-2D(image_in) point correspondences.)

  bool student_extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, 
                      const cv::Mat& camera_matrix, cv::Mat& rvec, 
                      cv::Mat& tvec, const std::string& config_folder){
  	
   	//std::cout << "Inside extrinsicCalib() in student_extrinsicCalib.cpp" << std::endl;
  	// Points are taken from user input
   	std::vector<cv::Point2f> image_points;
  	image_points = pickNPoints(4, img_in);

  	// open file to write [DO I ACTUALLY NEED TO WRITE?]
  	//std::string extrcalib_path = config_folder + "/extrinsicCalib.csv"; // where to save output
  	//std::ofstream output(extrcalib_path);
        	//if (!output.is_open()){
        	// throw std::runtime_error("Cannot write file: " + file_path);
        	//}
        	//for (const auto pt: image_points) {
        	//  output << pt.x << " " << pt.y << std::endl;
        	//}
        	//output.close();

  	// OpenCV solves the correspondance and load the rotation and translation vectors
  	cv::Mat dist_coeffs;
    dist_coeffs   = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
  	// This function returns the rotation and the translation vectors
  	// Object points are in 3D, image_points are in 2D (selected by user)
    bool done = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);
  	
  	if (!done) {
    	std::cerr << "FAILED SOLVE_PNP (extrinsicCalib)" << std::endl;
    }

    std::cout << "\tstudent_extrinsicCalib COMPLETED.\n" << std::flush;
  	return done;
  }

}
