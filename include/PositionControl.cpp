#include <PositionControl.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool PositionControl::threadInit()
{
	if(isRunning())
	{
		std::cout << "[ERROR] [POSITION CONTROL] threadInit(): "
		          << "A control thread is still running!\n";
		
		return false;
	}
	else
	{
		// Reset values
		this->isFinished = false;                                                           // New action started
		this->qRef = this->q;                                                               // Start from current joint position
		this->startTime = yarp::os::Time::now();                                            // Used to time the control loop
		
		return true; // jumps immediately to run()
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::threadRelease()
{
	send_joint_commands(const std::vector<double> this->q.data());                              // Maintain current joint positions
}

/*
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute instantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::compute_joint_limits(double &lower, double &upper, const int &i)
{
	// NOTE TO SELF: I need to update this with limits on the step size (i.e. velocity)
	
	lower = this->positionLimit[i][0] - this->qRef[i];
	upper = this->positionLimit[i][1] - this->qRef[i];
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve a discrete time step for Cartesian control                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,12,1> PositionControl::track_cartesian_trajectory(const double &time)
{
	// Variables used in this scope
	Eigen::Matrix<double,12,1> dx; dx.setZero();                                                // Value to be returned
	Eigen::Isometry3d pose;                                                                     // Desired pose
	Eigen::Matrix<double,6,1> vel, acc;
	
	if(this->isGrasping)
	{
		this->payloadTrajectory.get_state(pose,vel,acc,time);
		
		dx = this->G.transpose()*( this->dt*vel + this->K*pose_error(pose,this->payload.pose()) );		
	}
	else
	{
		if(this->leftControl)
		{
			this->leftTrajectory.get_state(pose,vel,acc,time);
			
			dx.head(6) = this->dt*vel + this->K*pose_error(pose,this->leftPose);
		}
		
		if(this->rightControl)
		{
			this->rightTrajectory.get_state(pose,vel,acc,time);
			
			dx.tail(6) = this->dt*vel + this->K*pose_error(pose,this->rightPose);
		}
	}
	
	return dx;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Solve the step size to track the joint trajectory                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd PositionControl::track_joint_trajectory(const double &time)
{
	Eigen::VectorXd dq(this->numJoints); dq.setZero();                                          // Value to be returned
	
	for(int i = 0; i < this->numJoints; i++) dq[i] = this->jointTrajectory[i].evaluatePoint(time) - this->q[i];
	
	return dq;
}
*/
