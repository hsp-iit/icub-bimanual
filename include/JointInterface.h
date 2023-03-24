    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     A class for interfacing with the joint motors on the iCub                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOINTINTERFACE_H_
#define JOINTINTERFACE_H_

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
		
		bool read_encoders();                                                               // As it says on the label
		
		bool send_joint_commands(const std::vector<double> &commands);
		
		std::vector<double> joint_positions() const  { return this->pos; }
		
		std::vector<double> joint_velocities() const { return this->vel; }
		
		void close();                                                                       // Close the device drivers & stop the robot
		
        protected:
        
		unsigned int numJoints;                                                             // Number of joints being controlled
		
		std::vector<std::array<double,2>> positionLimit;                                    // Upper and lower bounds on joint position
		std::vector<double>               velocityLimit;                                    // Absolute joint velocity
		
	private:
		
		std::vector<double> pos, vel;
			
	   	// These interface with the hardware on the robot itself
		yarp::dev::IControlLimits*   limits;                                                // Joint limits?
		yarp::dev::IControlMode*     mode;                                                  // Sets the control mode of the motor
		yarp::dev::IEncoders*        encoders;                                              // Joint position values (in degrees)
		yarp::dev::IPositionDirect*  pController;
		yarp::dev::PolyDriver        driver;                                                // Device driver
	
};                                                                                                  // Semicolon needed after class declaration

#endif
