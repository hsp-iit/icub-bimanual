
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Test build for code                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iCub2.h>
#include <iostream>

std::vector<std::string> portList = {"/icubSim/torso", "/icubSim/left_arm", "/icubSim/right_arm"};

int main(int argc, char *argv[])
{
	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 2)									
	{
		std::cerr << "[ERROR] [TEST BUILD] Path to urdf model required."
			  << " Usage: './grasping_demo ~/path/to/model.urdf'" << std::endl;
		return 1;                                                                           // Close with error
	}
	else
	{
		std::string file = argv[1];                                                         // Get the urdf model path
		iCub2 robot(file, jointList, portList);
		
		std::cout << "Success! All done." << std::endl;
	}
	
	return 0;
}
