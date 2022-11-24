  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Two-handed grasping demo with the iCub2.5                             //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iCub2.h>
#include <iCub2Configurations.h>
#include <yarp/os/RpcServer.h>

double long_time  = 5.0;
double short_time = 2.0;

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
		
		if(command == "aft")
		{
			output.addString("Indietro");
			robot.translate(yarp::sig::Vector({-0.075,0,0}),
			                yarp::sig::Vector({-0.075,0,0}),
			                short_time);
		}
		else if(command == "close")
		{
			output.addString("Arrivederci");
			robot.halt();                                                               // Stop any control threads
			active = false;
		}
		else if(command == "down")
		{
			output.addString("Giu'");
			yarp::sig::Vector left  = {0, 0, -0.075};
			yarp::sig::Vector right = {0, 0, -0.075};
			robot.translate(left,right,short_time);
		}
		else if(command == "fore")
		{
			output.addString("Avanti");
			robot.translate(yarp::sig::Vector({0.075,0,0}),
			                yarp::sig::Vector({0.075,0,0}),
			                short_time);
		}		
		else if(command == "grasp")
		{
			output.addString("Grazie");
			
			yarp::sig::Matrix left(4,4); left.eye();
			left[0][3] = 0.30;
			left[1][3] = 0.13;
			left[2][3] = 0.62;

			yarp::sig::Matrix right(4,4); right.eye();
			right[0][3] = 0.30;
			right[1][3] =-0.13;
			right[2][3] = 0.62;
			
			//robot.move_to_position(idealGrasp,short_time);
			robot.move_to_pose(left,right,short_time);
		}			
		else if(command == "home")
		{
			output.addString("Casa");
			robot.move_to_position(home, short_time);
		}
		else if(command == "in")
		{
			output.addString("Capito");
			yarp::sig::Vector left  = {0, -0.075, 0};
			yarp::sig::Vector right = {0,  0.075, 0};
			
			robot.translate(left, right, short_time);
		}
		else if(command == "left")
		{
			output.addString("Sinistra");
			robot.translate(yarp::sig::Vector({0,0.075,0.0}),
			                yarp::sig::Vector({0,0.075,0.0}),
			                short_time);
		}
		else if(command == "left hand pose")
		{
			output.addString("Check the other terminal.");
			robot.print_hand_pose("left");
		}
		else if(command == "out")
		{
			output.addString("Capito");
			yarp::sig::Vector left  = {0,  0.075, 0};
			yarp::sig::Vector right = {0, -0.075, 0};
			robot.translate(left,right, short_time);
		}
		else if(command == "ready")
		{
			output.addString("Pronto");
			robot.move_to_position(ready, short_time);
		}
		else if(command == "release")
		{
			output.addString("Capito");
			
			std::vector<yarp::sig::Vector> waypoints;
			waypoints.push_back(ready);
			waypoints.push_back(home);
			
			std::vector<double> times;
			times.push_back(2.0);
			times.push_back(3.0);
			
			robot.move_to_positions(waypoints, times);
		}			
		else if(command == "right")
		{
			output.addString("Destra");
			robot.translate(yarp::sig::Vector({0,-0.075,0}),
			                yarp::sig::Vector({0,-0.075,0}),
			                short_time);
		}
		else if(command == "right hand pose")
		{
			output.addString("Check the other terminal.");
			robot.print_hand_pose("right");
		}
		else if(command == "shake")
		{
			output.addString("Piacere");
			robot.move_to_position(shake, short_time);
		}
		else if(command == "stop")
		{
			output.addString("Fermare");
			robot.halt();
		}
		else if(command == "up")
		{
			output.addString("Su");
			yarp::sig::Vector left  = {0, 0, 0.075};
			yarp::sig::Vector right = {0, 0, 0.075};
			robot.translate(left,right,short_time);
		}
		else if(command == "wave")
		{
			output.addString("Ciao");
			
			std::vector<yarp::sig::Vector> wave;
			wave.push_back(wave1);
			wave.push_back(wave2);
			wave.push_back(wave1);
			wave.push_back(wave2);
			wave.push_back(home);
			
			std::vector<double> times;
			times.push_back(2.0);
			times.push_back(3.0);
			times.push_back(4.0);
			times.push_back(5.0);
			times.push_back(8.0);
			
			robot.move_to_positions(wave,times);
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
