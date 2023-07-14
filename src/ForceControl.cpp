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
			Eigen::Matrix<double,12,1> xddot = track_cartesian_trajectory(elapsedTime); // NOTE: Nonlinear effect Jdot*qdot already subtracted!
			
			Eigen::VectorXd redundantTask = this->kr*redundant_task() - this->qdot;     // Need to add damping in torque mode for stability
			
			Eigen::VectorXd lowerBound(this->numJoints), upperBound(this->numJoints);
			
			for(int i = 0; i < this->numJoints; i++) compute_joint_limits(lowerBound(i), upperBound(i), i);
			
			Eigen::VectorXd startPoint(this->numJoints);                                // This is needed for the QP solver
			
			if(QPSolver::last_solution_exists())
			{
				startPoint = QPSolver::last_solution().tail(this->numJoints);       // Remove any Lagrange multipliers
				
				for(int i = 0; i < this->numJoints; i++)
				{
					     if(startPoint(i) <= lowerBound(i)) startPoint(i) = lowerBound(i) + 0.01;
					else if(startPoint(i) >= upperBound(i)) startPoint(i) = upperBound(i) - 0.01;
				}				
			}
			else  startPoint = 0.5*(lowerBound + upperBound);                           // Half way between limits
			
			Eigen::VectorXd qddot;
			
			try
			{
				qddot = QPSolver::redundant_least_squares(this->invM*redundantTask, this->M, xddot, this->J, lowerBound, upperBound, startPoint);
			}
			catch(const std::exception &exception)
			{
				std::cerr << exception.what() << std::endl;
				
				qddot = -2*this->qdot;                                              // Try not to move
			}
			
			// Re-solve the problem subject to grasp constraints
			if(this->isGrasping)
			{
				// We need to set up a generic QP problem to solve for the
				// constrained control method
				
				Eigen::MatrixXd Jc = this->C*this->J;                               // Constraint matrix
				
				// This is annoying and I hate iDynTree for it
				Eigen::VectorXd Jdotqdot(12);
				Jdotqdot.head(6) = iDynTree::toEigen(this->computer.getFrameBiasAcc("left"));
				Jdotqdot.tail(6) = iDynTree::toEigen(this->computer.getFrameBiasAcc("right"));
				
				Eigen::VectorXd twist(12);
				twist.head(6) = this->leftTwist;
				twist.tail(6) = this->rightTwist;
				
				Eigen::VectorXd Jcdotqdot = this->C*Jdotqdot + this->Cdot*twist;
				
				// H = [ 0    Jc  ]
				//     [ Jc'   M  ]
				
				Eigen::MatrixXd H(6+this->numJoints,6+this->numJoints);
				H.block(              0, 0,              6,                6).setZero();
				H.block(              0, 6,              6,  this->numJoints) = Jc;
				H.block(this->numJoints, 6, this->numJoints,               6) = Jc.transpose();
				H.block(              6, 6, this->numJoints, this->numJoints) = this->M;
				
				// f = [ Jcdot *qdot ]
				//     [   - tau2    ]
				
				Eigen::VectorXd f(6+this->numJoints);
				f.head(6) = Jcdotqdot;
				f.tail(this->numJoints) = -this->M*qddot;                           // Needs to be converted to torque
				
				// B = [ -C'  0  ]
				//     [  C'  0  ]
				//     [  0  -I  ]
				//     [  0   I  ]
				
				Eigen::MatrixXd B(24+2*this->numJoints,6+this->numJoints);
				
				B.block(                 0, 0,                12,               6) = -this->C.transpose();
				B.block(                12, 0,                12,               6) =  this->C.transpose();
				B.block(                 0, 6,                24,              12).setZero();
				B.block(                24, 0, 2*this->numJoints, this->numJoints).setZero();
				B.block(                24, 6,   this->numJoints, this->numJoints) = -Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
				B.block(24+this->numJoints, 6,   this->numJoints, this->numJoints).setIdentity();
				
				// z = [   -fmax    ]
				//     [    fmin    ]
				//     [ -qddot_max ]
				//     [  qddot_min ]
				
				Eigen::VectorXd z(24+2*this->numJoints);
				
				formulate_grasp_constraints(fMin, fMax);
				
				z.block(                 0, 0,              12, 1) = -fMax;
				z.block(                12, 0,              12, 1) =  fMin;
				z.block(                24, 0, this->numJoints, 1) = -upperBound;
				z.block(24+this->numJoints, 0, this->numJoints, 1) =  lowerBound;
			}
			
			tau = this->M*qddot + this->coriolisAndGravity;				
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
		if(not this->leftTrajectory.get_state(pose,vel,acc,time))
		{
			std::cerr << "[ERROR] [FORCE CONTROL] track_cartesian_trajectory(): "
                                  << "Unable to obtain the desired state for the left hand.\n";
                }
                else xddot.head(6) = acc
                                   + this->D*(vel - iDynTree::toEigen(this->computer.getFrameVel("left")))
                                   + this->K*pose_error(pose,this->leftPose);
		
  		if(not this->rightTrajectory.get_state(pose,vel,acc,time))
  		{
  			std::cerr << "[ERROR] [FORCE CONTROL] track_cartesian_trajectory(): "
  			          << "Unable to obtain the desired state for the right hand.\n";
  		}
  		else xddot.tail(6) = acc
  		                   + this->D*(vel - iDynTree::toEigen(this->computer.getFrameVel("right")))
  		                   + this->K*pose_error(pose,this->rightPose);
	}
	
	// Subtract effect of nonlinear acceleration xddot - Jdot*qdot
	xddot.head(6) -= iDynTree::toEigen(this->computer.getFrameBiasAcc("left"));
	xddot.tail(6) -= iDynTree::toEigen(this->computer.getFrameBiasAcc("right"));
	
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

void ForceControl::formulate_grasp_constraint(Eigen::Matrix<double,12,1> &fMin, Eigen::Matrix<double,12,1> &fMax)
{
	Eigen::Vector3d leftAxis  =-this->leftPose.rotation().col(1);                               // Points out from back of hand, so we need to reverse it
	Eigen::Vector3d rightAxis = this->rightPose.rotation().col(1);                              // Points out of palm, leave as is
	
	
	for(int i = 0; i < 3; i++)
	{
		// Set the bounds so that the force is always point inward toward the object
		
		fMin(i) = std::min(0,this->graspForce*leftAxis(i));
		fMax(i) = std::max(0,this->graspForce*leftAxis(i));                                 
		
		fMin(i+6) = std::min(0,this->graspForce*rightAxis(i));
		fMax(i+6) = std::max(0,this->graspForce*rightAxis(i));
		
		// Bounds on the moments (not sure how to generate these?)
		fMin(i+3) = -1;
		fMax(i+3) =  1;
		fMin(i+9) = -1;
		fMax(i+9) =  1;
	}	
}
