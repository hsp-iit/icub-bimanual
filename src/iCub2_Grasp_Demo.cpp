  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Two-handed grasping demo with the iCub2.5                             //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iCub2.h>                                                                                  // Custom class for controlling iCub2
#include <map>                                                                                      // std::map
#include <Utilities.h>                                                                              // Functions for processing lists from yarp config files
#include <yarp/os/Property.h>                                                                       // Load configuration files
#include <yarp/os/RpcServer.h>                                                                      // Allows communication over yarp ports

std::map<std::string, JointTrajectory> jointConfigMap;                                              // Attach a name to each set of waypoints

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                             MAIN                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 4)
	{
		std::cerr << "[ERROR] [ICUB2 GRASP DEMO] Port name and path to URDF are required. "
		          << "Usage: ./icub2_grasp_demo /portPrefix /path/to/model.urdf /path/to/config.ini\n";
		
		return 1;                                                                           // Close with error
	}

	std::string portPrefix   = argv[1];                                                         // Get the port names
	std::string pathToURDF   = argv[2];                                                         // Get the file path
	std::string pathToConfig = argv[3];                                                         // Path to the configuration file
	
	// Generate port list prefixes
	std::vector<std::string> portList;
	portList.push_back(portPrefix + "/torso");
	portList.push_back(portPrefix + "/left_arm");
	portList.push_back(portPrefix + "/right_arm");
	
	yarp::os::Property parameter; parameter.fromConfigFile(pathToConfig);                       // Load the properties from the config file
		
	try // to start up the robot
	{
		// Get the joint list
		yarp::os::Bottle* bottle; bottle = parameter.find("joint_names").asList();
		if(bottle == nullptr)
		{
			std::cerr << "[ERROR] No list of joint names was specified in " << pathToConfig << ".\n";
			return 1;
		}
		std::vector<std::string> jointNames = string_from_bottle(bottle);

		// Load the joint predefined joint configurations
		bottle->clear(); bottle = &parameter.findGroup("JOINT_CONFIGURATIONS");
		if(bottle == nullptr)
		{
			std::cerr << "[ERROR] No group called JOINT_CONFIGURATIONS could be found in "
			          << pathToConfig << ".\n";
		}
		if(not load_joint_configurations(bottle,jointConfigMap)) return 1;
		
		iCub2 robot(pathToURDF, jointNames, portList);                                      // Start up the robot
		
		// Set the Cartesian gains
		double kp = parameter.findGroup("CARTESIAN_GAINS").find("proportional").asFloat64();
		double kd = parameter.findGroup("CARTESIAN_GAINS").find("derivative").asFloat64();	
		if(not robot.set_cartesian_gains(kp,kd)) return 1;
		
		// Set the joint gains
		kp = parameter.findGroup("JOINT_GAINS").find("proportional").asFloat64();
		kd = parameter.findGroup("JOINT_GAINS").find("derivative").asFloat64();	
		if(not robot.set_joint_gains(kp,kd)) return 1;
		
		// Configure communication across the yarp network
		yarp::os::Network yarp;                                                             // First connect to the network
		yarp::os::RpcServer port;                                                           // Create a port for sending / receiving info
		port.open("/command");                                                              // Open the port with the name '/command'
		yarp::os::Bottle input;                                                             // Store information from the user input
		yarp::os::Bottle output;                                                            // Store information to send to the user
		std::string command;                                                                // Response message, command from user
		
		// Run the control loop
		bool active = true;
		
		while(active)
		{
			output.clear();                                                             // Clear the output for the new loop
			port.read(input,true);                                                      // Read any new commands
			command = input.toString();                                                 // Convert to string
				
			auto iterator = jointConfigMap.find(command);
			if(iterator != jointConfigMap.end())
			{
				robot.move_to_positions(iterator->second.waypoints,
				                        iterator->second.times);

				output.addString("Capito");
			}
			else if(command == "close")
			{
				robot.halt();
				output.addString("Arrivederci");
				active = false;
			}
			else if(command == "stop")
			{
				robot.halt();
				output.addString("Fermata");
			}
			else
			{
				output.addString("Cosa");
			}
			
			port.reply(output);
		}
		
		robot.close();
		
		return 0;                                                                           // No problems with main
	}
	catch(std::exception &exception)
	{
		std::cerr << "[ERROR] [ICUB2 GRASP DEMO] There was a problem with initialization. "
		          << "See the error message below." << std::endl;
		          
		std::cout << exception.what() << std::endl;                                         // Inform the user
		
		return 1;                                                                           // Close with error
	}
}

/*
int main(int argc, char *argv[])
{

	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 3)									
	{
		std::cerr << "[ERROR] [ICUB2 GRASP DEMO] Port name and path to URDF are requred. "
			  << " Usage: './grasp-demo /portPrefix /path/to/model.urdf'" << std::endl;
		return 1;                                                                           // Close with error
	}
	else
	{
		std::string portPrefix   = argv[1];
		std::string pathToURDF = argv[2];
		
		std::vector<std::string> portList;
		portList.push_back(portPrefix + "/torso");
		portList.push_back(portPrefix + "/left_arm");
		portList.push_back(portPrefix + "/right_arm");
	
		// Create the robot model
		iCub2 robot(pathToURDF, jointNames, portList);
		
		// Configure communication across the yarp network
		yarp::os::Network yarp;                                                             // First connect to the network
		yarp::os::RpcServer port;                                                           // Create a port for sending / receiving info
		port.open("/command");                                                              // Open the port with the name '/command'
		yarp::os::Bottle input;                                                             // Store information from the user input
		yarp::os::Bottle output;                                                            // Store information to send to the user
		std::string command;                                                                // Response message, command from user
		
		// Run the control loop
		bool active = true;
		
		while(active)
		{
			output.clear();                                                             // Clear any previous information
			port.read(input,true);                                                      // Get the input from the /command port
			command = input.toString();                                                 // Convert to a string
			
			if(command == "aft")
			{
				
				output.addString("Indietro");
				
				robot.translate(Eigen::Vector3d(-0.075, 0.0, 0.0),
					        Eigen::Vector3d(-0.075, 0.0, 0.0),
					        short_time);
				
				yarp::os::Time::delay(short_time);
			}
			else if(command == "close")
			{
				output.addString("Arrivederci");
				robot.halt();                                                       // Stop any control threads
				active = false;
			}
			else if(command == "down")
			{
				output.addString("Giu'");
				
				robot.translate(Eigen::Vector3d(0.0, 0.0, -0.075),
					        Eigen::Vector3d(0.0, 0.0, -0.075),
					        short_time);
					       
				yarp::os::Time::delay(short_time);
			}
			else if(command == "fake grasp")
			{
				if(not robot.is_grasping())
				{
					output.addString("Grazie");
					
					robot.move_to_pose(Eigen::Isometry3d(Eigen::Translation3d(0.30, 0.15,0.65)),
							   Eigen::Isometry3d(Eigen::Translation3d(0.30,-0.15,0.65)),
							   short_time);
					
					yarp::os::Time::delay(short_time);
				}
				else	output.addString("Non posso!");
			}
			else if(command == "fore")
			{
				output.addString("Avanti");
				
				robot.translate(Eigen::Vector3d(0.075, 0.0, 0.0),
					        Eigen::Vector3d(0.075, 0.0, 0.0),
					        short_time);
					        
				yarp::os::Time::delay(short_time);
			}		
			else if(command == "grasp")
			{
				if(not robot.is_grasping())
				{
					output.addString("Grazie");
					
					robot.move_to_pose(Eigen::Isometry3d(Eigen::Translation3d(0.30, 0.15,0.65)),
							   Eigen::Isometry3d(Eigen::Translation3d(0.30,-0.15,0.65)),
							   short_time);
							   
					yarp::os::Time::delay(1.1*short_time);
					// Box is 295mm (0.295m) wide
					Eigen::Isometry3d boxPose(Eigen::Translation3d(0.3,0.0,0.65)); // Pose of box relative to robot
					
					robot.grasp_object( Payload( robot.left_hand_pose().inverse()*boxPose, mass, inertia ) );
				}
			}			
			else if(command == "home")
			{
				output.addString("Casa");
				
				robot.move_to_position(home, short_time);
				
				yarp::os::Time::delay(short_time);
			}
			else if(command == "in")
			{
				
	//			if(not robot.is_grasping() )
	//			{
					output.addString("Capito");
				
					robot.translate(Eigen::Vector3d(0.0,-0.075, 0.0),
							Eigen::Vector3d(0.0, 0.075, 0.0),
							short_time);
					
					yarp::os::Time::delay(short_time);     
	//			}
	//			else	output.addString("Non posso!");
			}
			else if(command == "left")
			{
				output.addString("Sinistra");
				
				robot.translate(Eigen::Vector3d(0.0, 0.075, 0.0),
					        Eigen::Vector3d(0.0, 0.075, 0.0),
					        short_time);
				
				yarp::os::Time::delay(short_time);     
			}
			else if(command == "left hand pose")
			{
				output.addString("Check the other terminal.");
				robot.print_hand_pose("left");
			}
			else if(command == "out")
			{
	//			if(not robot.is_grasping())
	//			{
					output.addString("Capito");
					
					robot.translate(Eigen::Vector3d(0.0, 0.075, 0.0),
							Eigen::Vector3d(0.0,-0.075, 0.0),
							short_time);
				
					yarp::os::Time::delay(short_time);
	//			}
	//			else	output.addString("Non posso!");
			}
			else if(command == "ready")
			{
				output.addString("Pronto");
				robot.move_to_position(ready, short_time);
				
				yarp::os::Time::delay(short_time);
			}
			else if(command == "release")
			{
				if(robot.is_grasping())
				{
					output.addString("Capito");
					
					robot.release_object();
					
					std::vector<Eigen::VectorXd> waypoints;
					waypoints.push_back(ready);
					waypoints.push_back(home);
					
					std::vector<double> times;
					times.push_back(2.0);
					times.push_back(4.0);
					
					robot.move_to_positions(waypoints,times);
					
					yarp::os::Time::delay(times.back());
				}

			}			
			else if(command == "right")
			{
				output.addString("Destra");
				
				robot.translate(Eigen::Vector3d(0.0,-0.075, 0.0),
					        Eigen::Vector3d(0.0,-0.075, 0.0),
					        short_time);
					        
				yarp::os::Time::delay(short_time);
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
				
				yarp::os::Time::delay(short_time);
			}
			else if(command == "stop")
			{
				output.addString("Fermare");
				robot.halt();
			}
			else if(command == "up")
			{
				output.addString("Su");

				robot.translate(Eigen::Vector3d(0.0, 0.0, 0.075),
					        Eigen::Vector3d(0.0, 0.0, 0.075),
					        short_time);
					        
				yarp::os::Time::delay(short_time);
			}
			else if(command == "wave")
			{
				output.addString("Ciao");
				
				std::vector<Eigen::VectorXd> wave;
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
				
				yarp::os::Time::delay(times.back());
			}
			else
			{
				output.addString("Cosa");
			}
			
			port.reply(output);                                                         // Send the output
		}
		
		robot.close();                                                                      // Close the device drivers

	return 0;
	}
}
*/
