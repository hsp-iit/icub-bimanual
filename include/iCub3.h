#include <iCubBase.h>

class iCub3 : public iCubBase
{
	public:
		iCub3(const std::string &pathToURDF,
		      const std::vector<std::string> &jointNames,
		      const std::vector<std::string> &portNames);
	
	private:
		void run();                                                                         // Main control loop
	
};                                                                                                  // Semicolon needed after class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Constructor for iCub3                                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
iCub3::iCub3(const std::string &pathToURDF,
             const std::vector<std::string> &jointNames,
             const std::vector<std::string> &portNames) :
             iCubBase(pathToURDF,
                      jointNames,
                      portNames,
                      iDynTree::Transform(iDynTree::Rotation::RPY(0,0,0),
                                          iDynTree::Position(0.0,0.0,0.65)) // NEED TO DOUBLE CHECK THIS VALUE
                     )
{
	// Lower the gains for velocity mode
	set_joint_gains(5.0, 0.01);                                                                 // Second argument for derivative gain doesn't matter
	set_cartesian_gains(10.0, 0.01);                         
}

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

