    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     A class for interfacing with the joint motors on the iCub                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOINTINTERFACE_H_
#define JOINTINTERFACE_H_

#include <Eigen/Core>                                                                               // Eigen::VectorXd
#include <iostream>                                                                                 // std::cerr, std::cout
#include <math.h>                                                                                   // M_PI
#include <string>                                                                                   // std::string
#include <vector>                                                                                   // std::vector
#include <yarp/dev/ControlBoardInterfaces.h>                                                        // I don't know what this does exactly...
#include <yarp/dev/PolyDriver.h>                                                                    // ... or this...
#include <yarp/os/Property.h>                                                                       // ... or this.

class JointInterface
{
	public:
		JointInterface(const std::vector<std::string> &jointList,
		               const std::vector<std::string> &portList);
		
		bool activate_control();                                                            // Activates control if construction successful
		
		bool read_encoders();                                                               // Update joint state internally
		
		bool send_joint_commands(const Eigen::VectorXd &commands);                          // As it says on the label
		
		Eigen::VectorXd joint_positions() const  { return this->pos; }
		
		Eigen::VectorXd joint_velocities() const { return this->vel; }
		
		void close();                                                                       // Close the device drivers & stop the robot
        
        protected:
        
//		enum ControlMode {position, force} controlMode;
		
		unsigned int numJoints;                                                             // Number of joints being controlled
		
		std::vector<std::array<double,2>> positionLimit;                                    // Upper and lower bounds on joint position
		std::vector<double>               velocityLimit;                                    // Absolute joint velocity
		
		
                                
	private:
		
		Eigen::VectorXd pos, vel;                                                           // Joint positions and velocities
			
	   	// These interface with the hardware on the robot itself
		yarp::dev::IControlLimits*   limits;                                                // Joint limits?
		yarp::dev::IControlMode*     mode;                                                  // Sets the control mode of the motor
		yarp::dev::IEncoders*        encoders;                                              // Joint position values (in degrees)
		yarp::dev::IPositionDirect*  pController;
		yarp::dev::PolyDriver        driver;                                                // Device driver
	
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
JointInterface::JointInterface(const std::vector<std::string> &jointList,
                               const std::vector<std::string> &portList)
                               :
                               numJoints(jointList.size()),
                               pos(Eigen::VectorXd::Zero(numJoints)),
                               vel(Eigen::VectorXd::Zero(numJoints))
{
	// Resize std::vector objects
	positionLimit.resize(this->numJoints);
	velocityLimit.resize(this->numJoints);

	////////////////////////// I copied this code from elsewhere ///////////////////////////////
	
	// Open up device drivers
	yarp::os::Property options;									
	options.put("device", "remotecontrolboardremapper");
	options.addGroup("axesNames");
	
	yarp::os::Bottle & bottle = options.findGroup("axesNames").addList();
	for(int i = 0; i < jointList.size(); i++) bottle.addString(jointList[i].c_str());           // Add the list of all the joint names
		
	yarp::os::Bottle remoteControlBoards;
	yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
	for(int i = 0; i < portList.size(); i++) remoteControlBoardsList.addString(portList[i]);    // Add the remote control board port names
	
	options.put("remoteControlBoards", remoteControlBoards.get(0));
	options.put("localPortPrefix", "/local");
	
	yarp::os::Property &remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
			    remoteControlBoardsOpts.put("writeStrict", "on");
			    
	////////////////////////////////////////////////////////////////////////////////////////////	
	
	if(not this->driver.open(options))
	{
		throw std::runtime_error("[ERROR] [JOINT INTERFACE] Constructor: Could not open device driver.");
	}
	else
	{
		if(not this->driver.view(this->pController))
		{
			throw std::runtime_error("[ERROR] [JOINT INTERFACE] Constructor: Unable to configure the position controller for the joint motors.");
		}
		else if(not this->driver.view(this->mode))
		{
			throw std::runtime_error("[ERROR] [JOINT_INTERFACE] Constructor: Unable to configure the control mode.");
		}
		else if(not this->driver.view(this->limits))
		{
			throw std::runtime_error("[ERROR] [JOINT INTERFACE] Constructor: Unable to obtain the joint limits.");
		}
		else
		{
			// Opened the motor controllers, so get the joint limits
			
			for(int i = 0; i < this->numJoints; i++)
			{
				double notUsed;
				this->limits->getLimits   (i, &this->positionLimit[i][0], &this->positionLimit[i][1]);
				this->limits->getVelLimits(i, &notUsed,                   &this->velocityLimit[i]);    // Assume vMin = -vMax
				
				// Convert values from degrees to radians
				this->positionLimit[i][0] *= M_PI/180.0;
				this->positionLimit[i][1] *= M_PI/180.0;
				this->velocityLimit[i]    *= M_PI/180.0;
			}
			
			// Finally, configure the encoders
			if(not this->driver.view(this->encoders))
			{
				throw std::runtime_error("[ERROR] [JOINT INTERFACE] Constructor: Unable to configure the encoders.");
			}
			else
			{
				double temp[this->numJoints];                                       // Temporary array to hold encoder values
				
				// Make 5 attempts to read the encoders
				for(int i = 0; i < 5; i++)
				{
					if(not this->encoders->getEncoders(temp) and i == 4)
					{
						throw std::runtime_error("[ERROR] [JOINT INTERFACE] Constructor: Could not obtain encoder values in 5 attempts.");
					}
				}
				
				// Success! We made it to the end
				for(int i = 0; i < this->numJoints; i++)
				{
					this->pos[i] = temp[i]*M_PI/180;                            // Assign initial joint values
					this->vel[i] = 0.0;
				}
			}		
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get new joint state information from the encoders                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool JointInterface::read_encoders()
{
	bool success = true;
	
	for(int i = 0; i < this->numJoints; i++)
	{
		success &= this->encoders->getEncoder     (i, &this->pos[i]);                       // Read the position
		success &= this->encoders->getEncoderSpeed(i, &this->vel[i]);                       // Read the velocity
		
		if(success)
		{
			this->pos[i] *= M_PI/180.0;                                                 // Convert to radians
			this->vel[i] *= M_PI/180.0;
		}
	}
	
	if(not success) std::cerr << "[ERROR] [JOINT INTERFACE] read_encoders(): "
		                  << "Could not obtain new encoder values.\n";
		        
	return success;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Send commands to the joint motors                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool JointInterface::send_joint_commands(const Eigen::VectorXd &commands)
{
	if(commands.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [JOINT INTERFACE] send_joint_command(): "
		          << "This robot has " << this->numJoints << " active joints but the input "
		          << "argument had " << commands.size() << " elements.\n";
		          
	        return false;
	}
	else
	{
		for(int i = 0; i < this->numJoints; i++)
		{
			if(not this->pController->setPosition(i,commands[i]*180.0/M_PI))
			{
				std::cerr << "[ERROR] [JOINT INTERFACE] send_joint_commands(): "
				          << "Could not send a command for joint " << i << ".\n";
				
				return false;
			}
		}
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Close the device interfaces on the robot                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
void JointInterface::close()
{
	for(int i = 0; i < this->numJoints; i++) this->mode->setControlMode(i, VOCAB_CM_POSITION);  // Set in position mode to lock the joint
	
	this->driver.close();                                                                       // Close the device drivers
}

#endif
