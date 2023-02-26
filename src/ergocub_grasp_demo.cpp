    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                    Demonstration of bimanual grasping with the ergoCub robot                  //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <ergoCub.h>
#include <ergoCubConfigurations.h>
#include <yarp/os/RpcServer.h>


// These are used for setting the length of trajetories
double long_time =  5.0;
double short_time = 2.0;

// These are used for creating a Payload object
double mass = 0.1;
Eigen::Matrix<double,3,3> inertia = (Eigen::MatrixXd(3,3) << 1e-06,   0.0,   0.0,
                                                               0.0, 1e-06,   0.0,
                                                               0.0,   0.0, 1e-06).finished();
                       
// List of joints to control                                           
std::vector<std::string> jointList = {"torso_roll", "torso_pitch", "torso_yaw",
			              "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_yaw", "l_wrist_roll", "l_wrist_pitch",
		                      "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_yaw", "r_wrist_roll", "r_wrist_pitch"};
		                      

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                              Main                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	// Ensure correct number of arguments
	if(argc != 3)
	{
		std::cerr << "[ERROR] [ERGOCUB GRASP DEMO] Path to URDF, and port name are required. "
                          << "Usage: './ergocub_grasp_demo /portName /path/to/model.urdf' " << std::endl;
                
                return 1;                                                                           // Close with error
	}
	else
	{
		std::string portName   = argv[1];
		std::string pathToURDF = argv[2];
		
		std::vector<std::string> portList;
		portList.push_back(portName + "/torso");
		portList.push_back(portName + "/left_arm");
		portList.push_back(portName + "/right_arm");
		
		ergoCub robot(pathToURDF, jointList, portList);                                     // Create the ergoCub controller
		
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
		
			if(command == "close")
			{
				output.addString("Arrivederci");
				active = false;
			}
			else if(command == "home")
			{
				output.addString("Casa");
				robot.move_to_position(home,short_time);
			}
			else if(command == "ready")
			{
				output.addString("Pronto");
				robot.move_to_position(ready,short_time);
			}
			else if(command == "shake")
			{
				output.addString("Piacere");
				robot.move_to_position(shake,short_time);
			}
			else if(command == "wave")
			{
				output.addString("Ciao");
				
//				robot.move_to_position(wave1,short_time);
				
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
			}
			else output.addString("Cosa");
			
			port.reply(output);
		}
		
		return 0;                                                                           // No problems with main
	}
}
