#include <ForceControl.h>


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool ForceControl::threadInit()
{
	std::cout << "Here we are in threadInit() so where is the damn problem???\n";
	
	if(isRunning())
	{
		std::cout << "[ERROR] [FORCE CONTROL] threadInit(): "
		          << "A control thread is still running!\n";
		return false;
	}
	else
	{
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
	std::cout << "WTF?!\n";
/*
	if(update_state())
	{
		std::cout << "What\n";
		
		double elapsedTime = yarp::os::Time::now() - this->startTime;                       // Time since start of control
		
		std::cout << "The\n";
		
		if(elapsedTime > this->endTime) this->isFinished = true;                            
		
		std::cout << "Fuck\n";
		
		Eigen::VectorXd tau(this->numJoints); tau.setZero();                                // We want to solve for this
		
		std::cout << "Why\n";
		
		if(this->controlSpace == joint)
		{
			std::cout << "Isn't\n";
			
			Eigen::VectorXd qddot = track_joint_trajectory(elapsedTime);
			
			std::cout << "This\n";

			iDynTree::Vector6 baseAcc; baseAcc.zero();                                       // No base acceleration
			iDynTree::LinkNetExternalWrenches wrench(this->computer.model()); wrench.zero(); // No external wrenches
			iDynTree::FreeFloatingGeneralizedTorques generalizedTorques;                     // This is where we store the result
			
			std::cout << "Working\n";
	
			this->computer.inverseDynamics(baseAcc, qddot, wrench, generalizedTorques); // Solve the inverse dynamics
			
			tau = iDynTree::toEigen(generalizedTorques.jointTorques());                 // Extract the joint torque vector
		}
		else // this->controlSpace == Cartesian
		{
			
		}

		std::cout << "Here are the joint torques:\n";
		std::cout << tau.transpose() << std::endl;
		
		if(not send_joint_commands(tau)) std::cerr << "[ERROR] [FORCE CONTROL] Coul dnot send joint commands for some reason.\n";
	}
	else
	{
		std::cerr << "[FLAGRANT SYSTEM ERROR] [FORCE CONTROL] run(): "
		          << "Unable to update the robot state for some reason.\n";
	}
*/
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void ForceControl::threadRelease()
{
	iDynTree::FreeFloatingGeneralizedTorques generalizedTorques;                                // Temporary storage
	
	this->computer.generalizedGravityForces(generalizedTorques);                                // Compute torques needed to negate gravity
	
	send_joint_commands(iDynTree::toEigen(generalizedTorques.jointTorques()));                  // Send to motors
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
	Eigen::VectorXd qddot(this->numJoints); qddot.setZero();                                    // Value to be returned
	
	double pos, vel, acc, lowerBound, upperBound;
	
	for(int i = 0; i < this->numJoints; i++)
	{
		pos = this->jointTrajectory[i].evaluatePoint(time,vel,acc);                         // Get the desired state from the trajectory generator
		
		qddot(i) = acc + this->kd*(vel - this->qdot(i)) + this->kp*(pos - this->q(i));
		
		compute_joint_limits(lowerBound,upperBound,i);
		
		     if(qddot(i) <= lowerBound) qddot(i) = lowerBound + 0.001;                      // Just above the lower bound
		else if(qddot(i) >= upperBound) qddot(i) = upperBound - 0.001;                      // Just below the upper bound
	}
	
	return qddot;
}
