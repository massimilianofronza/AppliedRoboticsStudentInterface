#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "student_functions.hpp"

#include <experimental/filesystem>
#include <sstream>


namespace student {

	
	void student_genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){ 

		static size_t img_no = 0;
  		static std::string folder_path = config_folder + "/camera_image/";
  		
		// creates the folder if it doesn't exist yet
		if (!std::experimental::filesystem::exists(folder_path)){		    

 	     		if(!std::experimental::filesystem::create_directories(folder_path)){
 			     	  throw std::logic_error( "FOLDER COULD NOT BE CREATED: " + folder_path);

 	     		}	
		}
  		
		// show image and wait for "save" request, or "quit"
	        cv::imshow(topic, img_in);
	        char c;
	        c = cv::waitKey(30);
	    
	        std::stringstream img_file;
            switch (c) {    	
			case 's':	
				{	
				// get the current time, in order to put this information in the image name
				time_t rawtime;
	  			struct tm * timeinfo;
	  			char buffer[80];
	  			time (&rawtime);
	  			timeinfo = localtime(&rawtime);
	  			strftime(buffer, sizeof(buffer),"%d-%m-%Y_%H:%M", timeinfo);
	  			std::string str(buffer);

	 			// save image
				img_file << folder_path << "img_" << str << "_" << std::setfill('0') 
						<< std::setw(3)  << (img_no++) << ".jpg";
			 	cv::imwrite( img_file.str(), img_in );

			 	std::cout << "Saved image " << img_file.str() << std::endl;
			 	break;
				}
			default:
				break;
    		}

	}

}
