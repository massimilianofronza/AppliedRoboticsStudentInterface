#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "visual_functions.hpp" 

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>
#include <sstream>


namespace student{

	/**
	* Implementation of the loadImage() function of the student_interface. It 
	* takes the imges from the config_folder/camera_image folder and loads them into
	* the simulator.
	*/
	void student_loadImage(cv::Mat& img_out, const std::string& config_folder){
		
		// Get a list of all images in config_folder/camera_image/
		static std::vector<cv::String> img_list; 
		const bool recursive = false;
    		cv::glob(config_folder + "/camera_image/*.jpg", img_list, recursive);
		
		// Current image being processed
		static size_t i=0;
  		static cv::Mat current_img;     

		// Read all images of the above specified folder
    		if(img_list.size() > 0){
			for (i=0; i < img_list.size(); i++){
				std::cout << "Reading image with index " << i << std::endl;
	     			current_img = cv::imread(img_list[i]);
				// Load the current image
				img_out = current_img;

				if(img_out.empty())
				{
					//std::cout << "Could not read the image: " << img_path << std::endl;
					throw std::logic_error( "STUDENT FUNCTION - No image loaded" );
				}
			}
		} else { 
			throw std::logic_error( "STUDENT FUNCTION - No images to load" );
		}			  

		// Load the current image
		//img_out = current_img;		

		
		return;
		
		  
	}
}
