
#include <iCubBase.h>

class iCub3 : public iCubBase
{
	public:
		iCub3(const std::string &fileName,
		      const std::vector<std::string> &jointList,
		      const std::vector<std::string> &portList);
	
	private:
		void run();                                                                         // Main control loop
	
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                          //     
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCub3::run()
{
	update_state();                                                                             // Update kinematics, dynamics
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since start of control loop
	
	Eigen::VectorXd vel = Eigen::VectorXd::Zero(this->n);                                       // We want to solve for this
	
	if(this->controlSpace == joint)
	{
		Eigen::VectorXd desiredVel = track_joint_trajectory(elapsedTime);
		
		vel = least_squares(desiredVel,                                                     // y
		                    Eigen::Matrix::Identity(this->n,this->n),                       // A
		                    this->lowerJointBound,                                          // xMin
		                    this->upperJointBound,                                          // xMax
		                    0.5*(this->lowerJointBound,this->upperJointBound));             // x0
	}
	else
	{
		Eigen::VectorXd xdot = track_cartesian_trajectory(elapsedTime);
		
		Eigen::VectorXf redundantTask;
		
		vel = least_squares(redundantTask,                                                  // xd
		                    this->M,                                                        // W
		                    this->J,                                                        // A
		                    xdot,                                                           // y
		                    this->lowerJointBound,                                          // xMin
		                    this->upperJointBound,                                          // xMax
		                    0.5*(this->lowerJointBound + this->upperJointBound));           // x0
	}
}

