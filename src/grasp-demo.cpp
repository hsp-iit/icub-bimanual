  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Two-handed grasping demo with the iCub2.5                             //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iCub2.h>
#include <yarp/os/RpcServer.h>

std::vector<std::string> portList = {"/icubSim/torso", "/icubSim/left_arm", "/icubSim/right_arm"};

std::vector<std::string> jointList = {"torso_pitch", "torso_roll", "torso_yaw",
			"l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw",
		        "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};

int main(int argc, char *argv[])
{

	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 2)									
	{
		std::cerr << "[ERROR] [GRASP DEMO] Path to urdf model required."
			  << " Usage: './grasp-demo /path/to/model.urdf'" << std::endl;
		return 1;                                                                           // Close with error
	}
	
	// Create the robot model
	std::string file = argv[1];
	iCub2 robot(file, jointList, portList);
	
	// Configure communication across the yarp network
	yarp::os::Network yarp;                                                                     // First connect to the network
	yarp::os::RpcServer port;                                                                   // Create a port for sending / receiving info
	port.open("/command");                                                                      // Open the port with the name '/command'
	yarp::os::Bottle input;                                                                     // Store information from the user input
	yarp::os::Bottle output;                                                                    // Store information to send to the user
	std::string command;                                                                        // Response message, command from user
	
	// Run the control loop
	
	bool active = true;
	
	while(active)
	{
		output.clear();                                                                     // Clear any previous information
		port.read(input,true);                                                              // Get the input from the /command port
		command = input.toString();                                                         // Convert to a string
		
		if(command == "close")
		{
			output.addString("Arrivederci");
			robot.halt();                                                               // Stop any control threads
			active = false;
		}
		else if(command == "hello")
		{
			output.addString("Ciao");
		}
		else if(command == "stop")
		{
			output.addString("Fermare");
			robot.halt();
		}
		else
		{
			output.addString("Cosa");
		}
		
		port.reply(output);                                                                 // Send the output
	}
	
	robot.close();                                                                              // Close the device drivers

	return 0;
}
