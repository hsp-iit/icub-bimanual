#include <ForceControl.h>


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool ForceControl::threadInit()
{
	if(isRunning())
	{
		std::cerr << "[ERROR] [FORCE CONTROL] threadInit(): A control thread is still running!\n";
		
		return false;
	}
	else
	{
		for(int i = 0; i < this->numJoints; i++)
		{
			if(not this->mode->setControlMode(i,VOCAB_CM_TORQUE))
			{
				std::cerr << "[ERROR] [FORCE CONTROL] threadInit(): Unable to activate torque control for joint " << i << ".\n";
	
				return false;
			}
		}
			
		// Reset values
		QPSolver::clear_last_solution();                                                    // Remove last solution
		this->isFinished = false;                                                           // New action started
		this->startTime = yarp::os::Time::now();                                            // Used to time the control loop
		return true;                                                                        // jumps immediately to run()
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
void ForceControl::run()
{
	if(update_state())
	{
		double elapsedTime = yarp::os::Time::now() - this->startTime;                       // Time since start of control
		
		if(elapsedTime > this->endTime) this->isFinished = true;                            
		
		Eigen::VectorXd tau(this->numJoints); tau.setZero();
		
		if(this->controlSpace == joint)
		{
			Eigen::VectorXd qddot = track_joint_trajectory(elapsedTime);
			
			tau = this->M*qddot + this->coriolisAndGravity;
		}
		else // this->controlSpace == Cartesian
		{
			
		}
		
		if(not send_joint_commands(tau)) std::cerr << "[ERROR] [FORCE CONTROL] run(): Could not send joint commands for some reason.\n";

	}
	else
	{
		std::cerr << "[FLAGRANT SYSTEM ERROR] [FORCE CONTROL] run(): "
		          << "Unable to update the robot state for some reason.\n";
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void ForceControl::threadRelease()
{
	for(int i = 0; i < this->numJoints; i++)
	{
		this->mode->setControlMode(i,VOCAB_CM_POSITION);
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute instantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool ForceControl::compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum)
{
	// NOTE TO FUTURE SELF: Need to compute a limit on the step size dq 
	
	if(jointNum > this->numJoints)
	{
		std::cerr << "[ERROR] [FORCE CONTROL] compute_joint_limits(): "
		          << "Range of joint indices is 0 to " << this->numJoints - 1 << ", "
		          << "but you called for " << jointNum << ".\n";

		return false;
	}
	else
	{
		lower = std::max( 2*(this->positionLimit[jointNum][0] - this->q[jointNum] - this->dt*this->qdot[jointNum])/(this->dt*this->dt),
	 	                   -(this->velocityLimit[jointNum] + this->qdot[jointNum])/this->dt );
		                   
		upper = std::min( 2*(this->positionLimit[jointNum][1] - this->q[jointNum] - this->dt*this->qdot[jointNum])/(this->dt*this->dt),
		                    (this->velocityLimit[jointNum] - this->qdot[jointNum])/this->dt );
		
		if(lower >= upper)
		{
			std::cerr << "[ERROR] [POSITION CONTROL] compute_joint_limits(): "
				  << "Lower limit " << lower << " is greater than upper limit " << upper << ". "
				  << "How did that happen???\n";
			
			return false;
		}
		else	return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Solve the Cartesian acceleration to track a trajectory                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,12,1> ForceControl::track_cartesian_trajectory(const double &time)
{
	// Variables used in this scope
	Eigen::Matrix<double,12,1> xddot; xddot.setZero();                                          // Value to be returned
	Eigen::Isometry3d pose;                                                                     // Desired pose
	Eigen::Matrix<double,6,1> vel, acc;

	if(this->isGrasping)
	{
		if(not this->payloadTrajectory.get_state(pose,vel,acc,time))
		{
			std::cerr << "[ERROR] [FORCE CONTROL] track_cartesian_trajectory(): "
			          << "Unable to obtain the desired state for the payload.\n";
		}
		else
		{
			xddot = this->G.transpose()*(acc + this->D*(vel - this->payload.twist()) + this->K*pose_error(pose,this->payload.pose()))
			      + this->Gdot.transpose()*this->payload.twist();
		}
	}
	else
	{
		Eigen::Matrix<double,12,1> xdot = this->J*this->qdot;                               
		
		if(not this->leftTrajectory.get_state(pose,vel,acc,time))
		{
			std::cerr << "[ERROR] [FORCE CONTROL] track_cartesian_trajectory(): "
                                  << "Unable to obtain the desired state for the left hand.\n";
                }
                else xddot.head(6) = acc + this->D*(vel - xdot.head(6)) + this->K*pose_error(pose,this->leftPose);
  
  		if(not this->rightTrajectory.get_state(pose,vel,acc,time))
  		{
  			std::cerr << "[ERROR] [FORCE CONTROL] track_cartesian_trajectory(): "
  			          << "Unable to obtain the desired state for the right hand.\n";
  		}
  		else xddot.tail(6) = acc + this->D*(vel - xdot.tail(6)) + this->K*pose_error(pose,this->rightPose);
	}
	
	return xddot;
}
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve the control to track a joint trajectory                          //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd ForceControl::track_joint_trajectory(const double &time)
{
	Eigen::VectorXd qddot(this->numJoints);
	
	double pos, vel, acc, lowerBound, upperBound;
	
	for(int i = 0; i < this->numJoints; i++)
	{
		pos = this->jointTrajectory[i].evaluatePoint(time,vel,acc);                         // Get the desired state from the trajectory generator
		
		qddot(i) = acc + this->kd*(vel - this->qdot(i)) + this->kp*(pos - this->q(i));
		
		if(not compute_joint_limits(lowerBound,upperBound,i))
		{
			std::cerr << "[ERROR] [FORCE CONTROL] track_joint_trajectory(): "
			          << "Unable to compute limits for joint " << i << ".\n";
			          
			qddot = -this->kd*this->qdot;                                               // Slow down
		}
		else
		{
			     if(qddot(i) <= lowerBound) qddot(i) = lowerBound + 0.001;              // Just above the lower bound
			else if(qddot(i) >= upperBound) qddot(i) = upperBound - 0.001;              // Just below the upper bound
		}
	}
	
	return qddot;
}
