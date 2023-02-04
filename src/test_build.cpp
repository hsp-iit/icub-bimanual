
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Test build for code                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iCub2.h>

std::vector<std::string> portList = {"/icubSim/torso", "/icubSim/left_arm", "/icubSim/right_arm"};

std::vector<std::string> jointList = {"torso_pitch", "torso_roll", "torso_yaw",
			"l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw",
		        "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};

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
