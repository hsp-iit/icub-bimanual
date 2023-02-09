    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                         Velocity control functions for the iCub/ergoCub                       //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ICUB_VELOCITY_H_
#define ICUB_VELOCITY_H_

#include <iCubBase.h>

class iCubVelocity : public iCubBase
{
	public:
		iCubVelocity(const std::string &pathToURDF,
			     const std::vector<std::string> &jointNames,
			     const std::vector<std::string> &portNames,
			     const Eigen::Isometry3d &_torsoPose) :
			     iCubBase(pathToURDF,
			              jointNames,
			              portNames,
			              _torsoPose) {}
			              
		// Inherited from the iCubBase class
		void compute_speed_limits(double &lower, double &upper, const int &i);
			     
		Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time);
		
		Eigen::VectorXd track_joint_trajectory(const double &time);
		
		// Inherited from yarp::PeriodicThread class
		void threadRelease();
			
};                                                                                                  // Semicolon needed after class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Solve velocity to track Cartesian trajectory                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,12,1> iCubVelocity::track_cartesian_trajectory(const double &time)
{
	// Variables used in this scope
	Eigen::Matrix<double,12,1> xdot; xdot.setZero();                                            // Value to be returned
	Eigen::Isometry3d pose;                                                                     // Desired pose
	Eigen::Matrix<double,6,1> vel, acc;                                                         // Desired velocity, accleration
	
	if(this->isGrasping)
	{
		if( this->payloadTrajectory.get_state(pose, vel, acc, time) )
		{
			xdot = this->G.transpose()*(vel + this->pose_error(pose, this->payload.pose()));
		}
	}
	else
	{
		if(this->leftControl)
		{
			this->leftTrajectory.get_state(pose, vel, acc, time);
			
			xdot.head(6) = vel + this->K*pose_error(pose, this->leftPose);
		}
		
		if(this->rightControl)
		{
			this->rightTrajectory.get_state(pose, vel, acc, time);
			
			xdot.tail(6) = vel + this->K*pose_error(pose, this->rightPose);
		}
	}
	
	return xdot;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve velocity to track joint trajectory                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd iCubVelocity::track_joint_trajectory(const double &time)
{
	Eigen::VectorXd ref(this->n);                                                               // Value to be returned
	double pos, vel, acc;                                                                       // Desired state along trajectory
	
	for(int i = 0; i < this->n; i++)
	{
		pos = this->jointTrajectory[i].evaluatePoint(time, vel, acc);                       // Get the desired state from the trajectory object
		
		ref[i] = vel + this->kp*(pos - this->q[i]);                                         // Compute feedforward + feedback control
	}
	
	return ref;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute instantaneous velocity limits on the joints                          //
///////////////////////////////////////////////////////////////////////////////////////////////////
void iCubVelocity::compute_speed_limits(double &lower, double &upper, const int &i)
{
	lower = std::max( (this->pLim[i][0] - this->q[i])/this->dt,                                 // Position limit
	        std::max( -this->vLim[i],                                                           // Velocity limit
	                  -sqrt(2*this->maxAcc*(this->q[i] - this->pLim[i][0]))));                  // Acceleration limit

	upper = std::min( (this->pLim[i][1] - this->q[i])/this->dt,
		std::min(  this->vLim[i],
		           sqrt(2*this->maxAcc*(this->pLim[i][1] - this->q[i]))));
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCubVelocity::threadRelease()
{
	std::vector<double> command;
	for(int i = 0; i < this->n; i++) command.push_back(0.0);
	send_velocity_commands(command);
}

#endif
