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
		QPSolver::clear_last_solution();                                                    // Remove last solution
		this->isFinished = false;                                                           // New action started
		this->qRef = this->q;                                                               // Start from current joint position
		this->startTime = yarp::os::Time::now();                                            // Used to time the control loop
		return true;                                                                        // jumps immediately to run()
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::run()
{
	if(update_state())
	{
		double elapsedTime = yarp::os::Time::now() - this->startTime;                       // Time since start of control
		
		Eigen::VectorXd dq(this->numJoints); dq.setZero();                                  // We want to solve for this		
		
		Eigen::VectorXd redundantTask = 0.1*(this->desiredConfiguration - this->q);
		
		if(this->controlSpace == joint)
		{
			Eigen::VectorXd desiredPosition(this->numJoints);                           // From the trajectory object
			Eigen::VectorXd lowerBound(this->numJoints);                                // Lower limit on joint motion
			Eigen::VectorXd upperBound(this->numJoints);                                // Upper limit on joint motion
				
			for(int i = 0; i < this->numJoints; i++)
			{
				desiredPosition(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime); // From the trajectory object
				
				compute_joint_limits(lowerBound(i),upperBound(i),i);                // Instantaneous limits on the joint step				
			}
			
			if(this->_robotModel == "iCub2")
			{
				// We need to run the QP solver to account for shoulder joint constraints
				
				Eigen::VectorXd startPoint;                                         // of the interior point method
				
				if(not QPSolver::last_solution_exists()) startPoint = 0.5*(lowerBound + upperBound); // Halfway between limits
				else
				{
					try // to get the last solution from the QP solver
					{
						startPoint = last_solution().tail(this->numJoints); // Remove Lagrange multipliers (if they exist)
						
						for(int i = 0; i < this->numJoints; i++)
						{
						             if(startPoint(i) < lowerBound(i)) startPoint(i) = lowerBound(i) + 0.001; // Just above the lower bound
							else if(startPoint(i) > upperBound(i)) startPoint(i) = upperBound(i) - 0.001; // Just below the upper bound
						}
					}
					catch(const std::exception &exception)
					{
						std::cout << exception.what() << std::endl;         // Print out the problem
						
						startPoint = 0.5*(lowerBound + upperBound);         // Set as halfway point, just to be safe
					}
				}
				
				// Now formulate constraints B*dq > z
				
				// B = [ -I ]
				//     [  I ]
				//     [  A ]
				Eigen::MatrixXd B(2*this->numJoints + 10, this->numJoints);
				B.block(                0, 0, this->numJoints, this->numJoints) = -Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
				B.block(  this->numJoints, 0, this->numJoints, this->numJoints) =  Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
				B.block(2*this->numJoints, 0,              10, this->numJoints) =  this->A; // Shoulder constraints
				
				// z = [   -dq_max  ]
				//     [    dq_min  ]
				//     [ -(A*q + b) ]
				Eigen::VectorXd z(2*this->numJoints + 10);
				z.block(              0, 0, this->numJoints, 1) = -upperBound;      // Upper limits on the joint motion
				z.block(this->numJoints, 0, this->numJoints, 1) =  lowerBound;      // Lower limits on the joint motion
				z.tail(10) = -(this->A*this->q + b);                                // Shoulder constraints on the joint motion
				
				try // to solve the QP problem
				{
					dq = QPSolver::solve(Eigen::MatrixXd::Identity(this->numJoints,this->numJoints),
					                     -(desiredPosition - this->qRef), B, z, startPoint);
				}
				catch(const std::exception &exception)
				{
					std::cout << exception.what() << std::endl;                 // Print out the problem
				}

			}
			else // this->robotModel == "ergoCub"
			{
				for(int i = 0; i < this->numJoints; i++)
				{
					dq(i) = desiredPosition(i) - this->q(i);                    // Difference between current position and desired position
					
					     if(dq(i) <= lowerBound(i)) dq(i) = lowerBound(i) + 0.001; // Just above the lower bound
					else if(dq(i) >= upperBound(i)) dq(i) = upperBound(i) - 0.001; // Just below the upper bound
				}
			}
		}
		else // this->controlSpace == Cartesian
		{
			Eigen::VectorXd dx = track_cartesian_trajectory(elapsedTime);              // Get the required Cartesian motion
			
			// Get the instantaneous limits on the joint motion
			Eigen::VectorXd lowerBound(this->numJoints), upperBound(this->numJoints);
			for(int i = 0; i < this->numJoints; i++)
			{
				compute_joint_limits(lowerBound(i),upperBound(i),i);
			}
			
			// Formulate the starting point for the QP solver
			Eigen::VectorXd startPoint;
			if(not QPSolver::last_solution_exists()) startPoint = 0.5*(lowerBound + upperBound); // Start half way
			else
			{
				try // to get the previous solution
				{
					startPoint = QPSolver::last_solution().tail(this->numJoints); // Remove any Lagrange multipliers that may exist
					
					for(int i = 0; i < this->numJoints; i++)
					{
						     if(startPoint(i) < lowerBound(i)) startPoint(i) = lowerBound(i) + 0.001;
						else if(startPoint(i) > upperBound(i)) startPoint(i) = upperBound(i) - 0.001;
					}
				}
				catch(const std::exception &exception)
				{
					std::cout << exception.what() << std::endl;                 // Print out the problem
					startPoint = 0.5*(lowerBound + upperBound);                 // Start in the middle for good measure
				}
			}
			
			double mu = sqrt((this->J*this->J.transpose()).determinant());              // Measure of manipulability
			double damping = sqrt((1 - mu/this->threshold)*this->maxDamping);           // Damping to be used in case of singularity
			
			// Now solve the control problem			
			if(this->_robotModel == "iCub2")
			{
				// NOTE: We need to solve a custom QP problem
				// because of shoulder constraints on iCub 2
									
				// z = [   -dq_max  ]
				//     [    dq_min  ]
				//     [ -(A*q + b) ]
				Eigen::VectorXd z(2*this->numJoints+10);
				z.block(              0, 0, this->numJoints, 1) = -upperBound;
				z.block(this->numJoints, 0, this->numJoints, 1) =  lowerBound;
				z.tail(10) = -(this->A*this->q + this->b);
			
				if(mu > this->threshold) // i.e. not singular
				{
					// H = [ 0  J ]
					//     [ J' M ]
					Eigen::MatrixXd H(12+this->numJoints, 12+this->numJoints);
					H.resize(12+this->numJoints,12+this->numJoints);
					H.block( 0, 0,              12,              12).setZero();
					H.block( 0,12,              12, this->numJoints) = this->J;
					H.block(12, 0, this->numJoints,              12) = this->J.transpose();
					H.block(12,12, this->numJoints, this->numJoints) = this->M;
					
					// f = [        -dx        ]
					//     [  -M*redundantTask ]
					Eigen::VectorXd f(12+this->numJoints);
					f.resize(12+this->numJoints);
					f.head(12)              = -dx;
					f.tail(this->numJoints) = -M*redundantTask;

					// B = [ 0 -I ]
					//     [ 0  I ]
					//     [ 0  A ]
					Eigen::MatrixXd B(2*this->numJoints+10,12+this->numJoints); // 2*n for joint limits, 10 for shoulder limits
					B.block(                0, 0,10+2*this->numJoints,             12) = Eigen::MatrixXd::Zero(10+2*this->numJoints,12);
					B.block(                0,12,     this->numJoints,this->numJoints) =-Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
					B.block(  this->numJoints,12,     this->numJoints,this->numJoints) = Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
					B.block(2*this->numJoints,12,                  10,this->numJoints) = this->A;
					
					try // to solve the QP problem
					{
						dq = QPSolver::solve(H,f,B,z,startPoint);
					}
					catch(const std::exception &exception)
					{
						std::cout << exception.what() << std::endl;         // Print the problem
					}
				}
				else // Solve Damped Least Squares (DLS)
				{
					// H = (J'*J + damping*I)
					Eigen::MatrixXd H = this->J.transpose()*this->J;
					for(int i = 0; i < this->numJoints; i++) H(i,i) += damping*damping; // The same as J'*J + damping^2 * I
					
					// f = -J'*dx
					Eigen::VectorXd f = -this->J.transpose()*dx;
					
					// B = [ -I ]
					//     [  I ]
					//     [  A ]
					Eigen::MatrixXd B(2*this->numJoints + 10, this->numJoints);
					B.block(                0,  0, this->numJoints, this->numJoints) = -Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
					B.block  (this->numJoints,  0, this->numJoints, this->numJoints) =  Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
					B.block(2*this->numJoints,  0,              10, this->numJoints) =  this->A;
					
					try // to solve the QP problem
					{
						dq = QPSolver::solve(H,f,B,z,startPoint);
					}
					catch(const std::exception &exception)
					{
						std::cout << exception.what() << std::endl;         // Print out the problem
					}
				}
			}
			else // this->_robotModel = "ergoCub")
			{	
				if(mu > this->threshold) // i.e. not singular
				{
					try // to solve the QP problem
					{
						dq = QPSolver::least_squares(redundantTask, this->M, dx, this->J,
					                                     lowerBound, upperBound, startPoint); // too easy lol
					}
					catch(const std::exception &exception)
					{
						std::cout << exception.what() << std::endl;
					}
				}                             
				else // Solve Damped Least Squares (DLS)
				{
					// v = [  0 ]
					//     [ dx ]
					Eigen::VectorXd v(this->numJoints + 12);
					v.head(this->numJoints).setZero();
					v.tail(12) = dx;
					
					// P = [ damping*I ]
					//     [     J     ]
					Eigen::MatrixXd P(this->numJoints+12,this->numJoints);
					P.block(              0, 0, this->numJoints, this->numJoints) = damping*Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
					P.block(this->numJoints, 0,              12, this->numJoints) = this->J;
					
					try // to solve the QP problem
					{
						dq = QPSolver::least_squares(v,P, Eigen::MatrixXd::Identity(12+this->numJoints,12+this->numJoints),
						                             lowerBound,upperBound,startPoint);
					}
					catch(const std::exception &exception)
					{
						std::cout << exception.what() << std::endl;
					}
				}	
			}
		}
	
		this->qRef += dq;                                                                   // Update reference position for joint motors
		
		if(not send_joint_commands(qRef)) std::cout << "[ERROR] [POSITION CONTROL] Could not send joint commands for some reason.\n";
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::threadRelease()
{
	send_joint_commands(this->q);                                                               // Maintain current joint positions
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute instantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool PositionControl::compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum)
{
	// NOTE TO FUTURE SELF: Need to compute a limit on the step size dq 
	
	if(jointNum > this->numJoints)
	{
		std::cerr << "[ERROR] [POSITION CONTROL] compute_joint_limits(): "
		          << "Range of joint indices is 0 to " << this->numJoints - 1 << ", "
		          << "but you called for " << jointNum << ".\n";

		return false;
	}
	else
	{
		lower = this->positionLimit[jointNum][0] - this->qRef[jointNum];
		upper = this->positionLimit[jointNum][1] - this->qRef[jointNum];
		
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
 //                        Solve a discrete time step for Cartesian control                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,12,1> PositionControl::track_cartesian_trajectory(const double &time)
{
	// NOTE TO FUTURE SELF:
	// There are no checks here to see if the trajectory is queried correctly.
	// This could cause problems later
	
	// Variables used in this scope
	Eigen::Matrix<double,12,1> dx; dx.setZero();                                                // Value to be returned
	Eigen::Isometry3d pose;                                                                     // Desired pose
	Eigen::Matrix<double,6,1> vel, acc;                                                         // Desired velocity & acceleration
	
	if(this->isGrasping)
	{
		this->payloadTrajectory.get_state(pose,vel,acc,time);                               // Get the desired object state for the given time              
		
		dx = this->G.transpose()*(this->dt*vel + this->K*pose_error(pose,this->payload.pose())); // Feedforward + feedback control		
	}
	else
	{
		this->leftTrajectory.get_state(pose,vel,acc,time);                                  // Desired state for the left hand
		dx.head(6) = this->dt*vel + this->K*pose_error(pose,this->leftPose);                // Feedforward + feedback on the left hand

		this->rightTrajectory.get_state(pose,vel,acc,time);                                 // Desired state for the right hand
		dx.tail(6) = this->dt*vel + this->K*pose_error(pose,this->rightPose);               // Feedforward + feedback on the right hand
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

