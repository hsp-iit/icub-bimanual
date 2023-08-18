    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //        A class that defines the dynamic properties of an object being carried by a robot       //   
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PAYLOAD_H_
#define PAYLOAD_H_

#include <Eigen/Geometry>                                                                           // Eigen::Isometry3d
#include <iostream>                                                                                 // std::cout, std::cerr

class Payload
{
	public:
		Payload() : Payload(Eigen::Isometry3d(Eigen::Translation3d(0,0,0)*Eigen::Quaterniond(1,0,0,0))) {}
		
		Payload(const Eigen::Isometry3d &localPose) : _localPose(localPose) {}              // Local pose relative to some contact point on the hand
		
		Eigen::Isometry3d pose() const { return this->_globalPose; }                        // Get the actual pose of the object
		
		Eigen::Isometry3d local_pose() const { return this->_localPose; }                   // Get the local pose from contact point to center of object
		
		Eigen::Matrix<double,6,1> twist() const { return this->_twist; }                    // Get the linear & angular velocity of the object
		
		void update_state(const Eigen::Isometry3d &globalToLocal,                           // Pose at the contact point in global frame
		                  const Eigen::Matrix<double,6,1> &contactTwist);                   // Twist at the contact point       
		                  
		
	private:
		
		Eigen::Isometry3d _localPose;                                                       // Transform from contact point to centre of object
		                                                    
		Eigen::Isometry3d _globalPose;                                                      // Pose in the global frame                                      
		
		Eigen::Matrix<double,6,1> _twist;                                                   // Linear & angular velocity
	
};                                                                                                  // Semicolon needed after class declaration

#endif
