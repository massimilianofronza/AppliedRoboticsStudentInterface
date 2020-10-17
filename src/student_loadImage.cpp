#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "student_functions.hpp" 

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>
#include <sstream>


namespace student{

	void student_loadImage(cv::Mat& img_out, const std::string& config_folder){

		//Load the first image from /tmp/camera_image000, saved there by student_genericImageListener
		std::stringstream ss;
        	ss << config_folder << "/camera_image000/000.jpg";
		std::string img_path = ss.str();
		
		std::cout << "Loading image " << img_path << std::endl;
		
		img_out = cv::imread(img_path);
		  

		if(img_out.empty())
		{
			std::cout << "Could not read the image: " << img_path << std::endl;
			throw std::logic_error( "STUDENT FUNCTION - No image loaded" );
		}

		
		  
	}
}
