    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                    Enables interfacing with the iCub/ergoCub through YARP                     //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <CommandInterface.h>                                                                       // thrift-generated class
#include <iostream>                                                                                 // std::cerr, std::cout
#include <map>                                                                                      // std::map
#include <PositionControl.h>                                                                        // For control of ergoCub, iCub robots
#include <Utilities.h>                                                                              // JointTrajectory object structure
#include <yarp/os/Property.h>                                                                       // Load configuration files
#include <yarp/os/RpcServer.h>                                                                      // Allows communication over yarp ports

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Overrides the functions specified in CommandInterface.thrift                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
class CommandServer : public CommandInterface
{
	public:
	
		CommandServer(PositionControl *_robot,
		              std::map<std::string,JointTrajectory> *_jointActionMap,
		              std::map<std::string,CartesianMotion> *_leftHandMap,
		              std::map<std::string,CartesianMotion> *_rightHandMap)
		             :
		             robot(_robot),
		             jointActionMap(_jointActionMap),
		             leftHandMap(_leftHandMap),
		             rightHandMap(_rightHandMap) {}        
		
		////////// NOTE: These are directly copied from CommandInterface.h //////////
		
		// Move the hands to grasp position
		bool grasp_object(const std::vector<double>& handPoses)
		{
			if(handPoses.size() == 12)
			{
				return true;
			}
			else			
			{
				std::cerr << "[ERROR] [iCUB COMMAND SERVER] grasp_object(): "
					  << "Expected 12 elements for the input, but it had "
					  << handPoses.size() << ".\n";
					  
				return false;
			}
		}

		// Query whether the robot has finished executing an action
		bool is_finished() { return this->robot->is_finished(); }

		// Move a grasped object to a pose
		bool move_object_to_pose(const std::vector<double>& pose, const double time)
		{
			if(pose.size() == 6)
			{
				Eigen::Isometry3d desiredPose;
				
				try
				{
					desiredPose = transform_from_vector(pose);
					
					return this->robot->move_object(desiredPose,time);
				}
				catch(const std::exception &exception)
				{
					std::cout << exception.what() << std::endl;                 // Print out the problem
					
					return false;
				}
			}
			else
			{
				std::cerr << "[ERROR] [iCUB CONTROL SERVER] move_object_to_pose(): "
				          << "Expected 6 elements for the argument, but it had " << pose.size() << ".\n";
				
				return false;
			}
		}

		// Move the hands to given poses
		bool move_hands_to_pose(const std::vector<double>& poses, const double time)
		{
			if(poses.size() == 12)
			{
				Eigen::Isometry3d leftPose, rightPose;
				
				try
				{
					leftPose = transform_from_vector({poses.begin(),poses.begin()+6});
					
					rightPose = transform_from_vector({poses.begin()+6,poses.begin()+12});
					
					return this->robot->move_to_pose(leftPose, rightPose, time);
				}
				catch(std::exception &exception)
				{
					std::cout << exception.what() << std::endl;
					
					return false;
				}
			}
			else
			{
				std::cerr << "[ERROR] [iCUB COMMAND SERVER] move_hands_to_pose(): "
				          << "Expected 12 elements for the argument, but it had " << poses.size() << ".\n";
				
				return false;
			}
		}

		// Move hands by a prescribed action
		bool move_hands_by_action(const std::string& actionName)
		{
			// Variables used in this scope
			std::vector<Eigen::Isometry3d> leftWaypoints, rightWaypoints;
			std::vector<double> times;
			Type type;
			
			auto temp = this->leftHandMap->find(actionName);
			
			if(temp == this->leftHandMap->end())
			{
				std::cerr << "[ERROR] [iCUB COMMAND SERVER] move_hands_by_action(): "
				          << "Could not find the action named " << actionName
				          << " for the left hand.\n";
				
				return false;
			}
			else
			{
				leftWaypoints = temp->second.waypoints;
				times = temp->second.times;
				type = temp->second.type;
			}
			
			temp = this->rightHandMap->find(actionName);
			
			if(temp == this->rightHandMap->end())
			{
				std::cerr << "[ERROR] [iCUB COMMAND SERVER] move_hands_by_action(): "
				          << "Could not find the action named " << actionName
				          << " for the right hand.\n";
				
				return false;
			}
			else	rightWaypoints = temp->second.waypoints;
			
			if(type == absolute)
			{
				std::cout << "NOT YET PROGRAMMED.\n";
				return false;
			}
			else // type == relative
			{
				Eigen::Isometry3d desiredLeft, desiredRight;
				
				try
				{
					desiredLeft  = this->robot->hand_pose("left")*leftWaypoints[0];
					desiredRight = this->robot->hand_pose("right")*rightWaypoints[0];
					
					this->robot->move_to_pose(desiredLeft,desiredRight,times[0]);
					
					return true;
				}
				catch(const std::exception &exception)
				{
					std::cout << exception.what() << std::endl;
					return false;
				}
			}        
		}

		// Move the joints to a prescribed joint configuration
		bool move_to_configuration(const std::vector<double>& jointConfiguration, const double time)
		{
			return this->robot->move_to_position(Eigen::VectorXd::Map(&jointConfiguration[0], jointConfiguration.size()), time);
		}
			
		// Move the joints to a prescribed joint position
		bool move_to_named_configuration(const std::string& configName)
		{
			auto jointConfig = this->jointActionMap->find(configName);
			
			if(jointConfig == this->jointActionMap->end())
			{
				std::cerr << "[ERROR] [iCUB COMMAND SERVER] move_to_named_configuration(): "
				          << "Could not find a joint configuration named "
				          << configName << " in the list.\n";
				
				return false;
			}
			else
			{
				return this->robot->move_to_positions(jointConfig->second.waypoints,
				                                      jointConfig->second.times);
			}
		}	

		// Release an object that has been grasped
		bool release_object() { return this->robot->release_object(); }

		// Stop the robot moving immediately
		bool stop()
		{
			// NOTE TO SELF: I should either set as void, or do some logic check...
			this->robot->halt();
			return true;
		}	
	    
	private:
		
		PositionControl *robot;                                                             // Pointer to robot object
		
		std::map<std::string, JointTrajectory> *jointActionMap;                             // Map of prescribed joint configurations

		std::map<std::string, CartesianMotion> *leftHandMap, *rightHandMap;
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                             MAIN                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	std::string errorMessage = "[ERROR] [iCUB COMMAND SERVER] ";
	
	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 4)
	{
		std::cerr << errorMessage << "Port name and path to URDF are required. "
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
		// Get the joint list from the config file
		yarp::os::Bottle* bottle; bottle = parameter.find("joint_names").asList();
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "No list of joint names was specified in " << pathToConfig << ".\n";
			return 1;
		}
		
		std::vector<std::string> jointNames = string_from_bottle(bottle);                   // Function specified in Utils.h

		// Load the joint-space actions from the config file
		std::map<std::string, JointTrajectory> jointActionMap;                              // NOTE: JointTrajectory is a data structure in Utils.H
		
		bottle->clear(); bottle = &parameter.findGroup("JOINT_SPACE_ACTIONS");
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "No group called JOINT_SPACE_ACTIONS could be found in "
			          << pathToConfig << ".\n";
			
			return 1;
		}
		if(not load_joint_configurations(bottle,jointActionMap)) return 1;
		
		// Load the left hand actions from the config file
		
		bottle->clear(); bottle = parameter.findGroup("CARTESIAN_ACTIONS").find("names").asList();
		
		std::vector<std::string> nameList = string_from_bottle(bottle);
		
		std::map<std::string, CartesianMotion> leftHandMap;
		
		bottle->clear(); bottle = parameter.findGroup("CARTESIAN_ACTIONS").find("left").asList();
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "No list called 'left' in the CARTESIAN_ACTIONS group "
			                          << "could be found in " << pathToConfig << ".\n";
			return 1;
		}
		
		if(not load_cartesian_trajectories(bottle,nameList,leftHandMap)) return 1;
		
		// Load the right hand actions from the config file
		std::map<std::string, CartesianMotion> rightHandMap;
		
		bottle->clear(); bottle = parameter.findGroup("CARTESIAN_ACTIONS").find("right").asList();
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "No list called 'right' in the CARTESIAN_ACTIONS group "
			                          << "could be found in " << pathToConfig << ".\n";
			return 1;
		}
		
		if(not load_cartesian_trajectories(bottle,nameList,rightHandMap)) return 1;
		
		/************** NEED TO LOAD NAME AS AN ARGUMENT **********************/
		PositionControl robot(pathToURDF, jointNames, portList, "iCub2");                   // Start up the robot
		
		// Set the Cartesian gains
		double kp = parameter.findGroup("CARTESIAN_GAINS").find("proportional").asFloat64();
		double kd = parameter.findGroup("CARTESIAN_GAINS").find("derivative").asFloat64();	
		if(not robot.set_cartesian_gains(kp,kd)) return 1;
		
		// Set the joint gains
		kp = parameter.findGroup("JOINT_GAINS").find("proportional").asFloat64();
		kd = parameter.findGroup("JOINT_GAINS").find("derivative").asFloat64();	
		if(not robot.set_joint_gains(kp,kd)) return 1;
		
		// Establish communication over YARP
		yarp::os::Network yarp;
		yarp::os::Port port;
		CommandServer commandServer(&robot, &jointActionMap, &leftHandMap, &rightHandMap);  // Create command server
		commandServer.yarp().attachAsServer(port);
		
		if(not port.open("/commandServer"))
		{
			std::cerr << errorMessage << "Could not open port /commandServer.\n";
			return 1;
		}
		
		while(true)
		{
			std::cout << "\nWorker bees can leave.\n";
			yarp::os::Time::delay(5);
			std::cout << "Even drones can fly away.\n";
			yarp::os::Time::delay(5);
			std::cout << "The Queen is their slave.\n";
			yarp::os::Time::delay(20);
		}
		
		port.close();
		
		robot.close();
		
		return 0;                                                                           // No problems with main
	}
	catch(std::exception &exception)
	{
		std::cerr << errorMessage << "There was a problem with initialization.\n";
		          
		std::cout << exception.what() << std::endl;                                         // Inform the user
		
		return 1;                                                                           // Close with error
	}
}
