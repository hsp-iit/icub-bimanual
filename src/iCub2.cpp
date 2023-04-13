#include <iCub2.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Constructor for the iCub 2                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCub2::iCub2(const std::string &pathToURDF,
             const std::vector<std::string> &jointNames,
             const std::vector<std::string> &portNames)
             :
             PositionControl(pathToURDF,
                             jointNames,
                             portNames,
                             Eigen::Isometry3d(Eigen::Translation3d(0.0,0.0,0.63)*Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())),
                             "iCub2")
{
	// Set the constraints for the iCub2 shoulder tendons.
	// A *single* arm is constrained by
	//      A*q + b > 0,
	// but we have two arms so we need to double up the constraint matrix.
	
	double c = 1.71;
	this->A = Eigen::MatrixXd::Zero(10,this->numJoints);
	this->A.block(0,3,5,3) <<  c, -c,  0,
	                           c, -c, -c,
	                           0,  1,  1,
	                          -c,  c,  c,
	                           0, -1, -1;
	                           
	this->A.block(5,10,5,3) = this->A.block(0,3,5,3);                                           // Same constraint for right arm as left arm
	
	this->b.head(5) << 347.00*(M_PI/180),
	                   366.57*(M_PI/180),
	                    66.60*(M_PI/180),
	                   112.42*(M_PI/180),
	                   213.30*(M_PI/180);
	                   
	this->b.tail(5) = this->b.head(5);                                                          // Same constraint for the right arm
	
	// In discrete time we have:
	// A*(q + dq) >= b ---> A*dq > -(A*q + b)
	
	// The constraint matrix is unique to iCub2.
	// NOTE: First column here pertains to Lagrange multipliers when running in Cartesian mode
	// B = [ 0   -I ]
	//     [ 0    I ]
	//     [ 0    A ]
	this->B.resize(10+2*this->numJoints,12+this->numJoints);                                    // 2*n for joint limits, 10 for shoulder limits
	this->B.block(                0, 0,10+2*this->numJoints,             12) = Eigen::MatrixXd::Zero(10+2*this->numJoints,12);
	this->B.block(                0,12,     this->numJoints,this->numJoints) =-Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
	this->B.block(  this->numJoints,12,     this->numJoints,this->numJoints) = Eigen::MatrixXd::Identity(this->numJoints,this->numJoints);
	this->B.block(2*this->numJoints,12,                  10,this->numJoints) = this->A;
	
	this->z.resize(10+2*this->numJoints);
	
	// Set the desired configuration for the arms when running in Cartesian mode
	this->setPoint.resize(this->numJoints);
	this->setPoint.head(3).setZero();                                                           // Torso joints -> stay upright if possible
	this->setPoint(3) = -0.5;
	this->setPoint(4) =  0.5;
	this->setPoint(5) =  0.5;
	this->setPoint(6) =  0.9;                                                                   // Elbow
	this->setPoint(7) = -0.5;
	this->setPoint(8) = -0.1;
	this->setPoint(9) =  0.0;
	this->setPoint.tail(7) = this->setPoint.block(3,0,7,1);

}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCub2::run()
{
	if(update_state())
	{
		double elapsedTime = yarp::os::Time::now() - this->startTime;                       // Time since start of the control loop
		
		Eigen::VectorXd lowerBound(this->numJoints), upperBound(this->numJoints);           // Instantaneous bounds on joint motion
		
		if(this->controlSpace == joint)
		{
			Eigen::VectorXd q_d(this->numJoints);                                       // Desired joint positions
			
			// Get the desired position from the joint trajectory generator
			// and set the bounds on the solution
			for(int i = 0; i < this->numJoints; i++)
			{
				q_d(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime);       // Desired position for the given time
				
				// Get the limits on the solution
				lowerBound(i) = this->positionLimit[i][0];
				upperBound(i) = this->positionLimit[i][1];
				
				// Convert to constraint vector for the QP solver
				this->z(i)                 = -upperBound(i);
				this->z(i+this->numJoints) =  lowerBound(i);
			}
				this->z.tail(10) = -this->b;                                        // For the shoulder constraints
				
			// Get the start point for the QP solver
			Eigen::VectorXd startPoint;
		
			if(not last_solution_exists()) startPoint = 0.5*(lowerBound + upperBound);  // Halfway between limits
			else                                                                        // Use last solution to speed up QP solver
			{
				try // to get the previous solution from the solver
				{
					startPoint = last_solution().tail(this->numJoints);         // Remove Lagrange multipliers
					
					// Ensure within limits since they may have changed
					for(int i = 0; i < this->numJoints; i++)
					{
						     if(startPoint(i) > upperBound(i)) startPoint(i) = upperBound(i) - 1e-03;
						else if(startPoint(i) < lowerBound(i)) startPoint(i) = lowerBound(i) + 1e-03;
					}
				}
				catch(const std::exception &exception)
				{
					std::cout << exception.what() << std::endl;                 // Failed for some reason
				}			
			}
			
			try // to solve the QP problem
			{
				qRef = solve(Eigen::MatrixXd::Identity(this->numJoints,this->numJoints), // H
					    -q_d,                                                        // f
					     this->B.block(0,12,10+2*this->numJoints,this->numJoints),   // Remove component for Lagrange multipliers in Cartesian mode
					     this->z,                                                    // Constraint vector
					     startPoint);                                                // x0
			}
			catch(const std::exception &exception)
			{
				std::cout << exception.what() << std::endl;                         // Inform the user
			}
		}
		else // this->controlSpace == cartesian
		{
			Eigen::VectorXd dx = track_cartesian_trajectory(elapsedTime);               // Get the feedforward + feedback control
			
			Eigen::VectorXd redundantTask = 0.05*(this->setPoint - this->q);            // As it says
			
			Eigen::VectorXd dq(this->numJoints);                                        // We want to solve for this
			
			// Solve for the instantaneous joint limits
			for(int i = 0; i < this->numJoints; i++)
			{
				compute_joint_limits(lowerBound(i),upperBound(i),i);
				this->z(i)                 = -upperBound(i);
				this->z(i+this->numJoints) =  lowerBound(i);
			}
				this->z.tail(10) = -(this->A*this->qRef + this->b);
				
			// Values needed for the QP solver
			Eigen::MatrixXd H;
			Eigen::VectorXd f;
			Eigen::VectorXd startPoint;
				
			double mu = sqrt((this->J*this->J.transpose()).determinant());
			double threshold = 1e-05;
			
			if( mu < threshold ) // Solve the damped least squares problem
			{
				double maxDamping = 0.1;
				double damping    = (1 - (mu*mu)/(threshold*threshold))*maxDamping;
				
				H = this->J.transpose()*this->J;
				for(int i = 0; i < this->numJoints; i++) H(i,i) += damping;         //  The same as J'*J + lambda*I
				
				f = -this->J.transpose()*dx;
				
				if(last_solution_exists())
				{
					try
					{
						startPoint = last_solution().tail(this->numJoints);  // Ignore any Lagrange multipliers
						
						// Ensure it's within the joint limits
						for(int i = 0; i < this->numJoints; i++)
						{
							     if(startPoint(i) < lowerBound(i)) startPoint(i) += 0.001;
							else if(startPoint(i) > upperBound(i)) startPoint(i) -= 0.001;
						}
					}
					catch(const std::exception &exception)
					{
						std::cout << exception.what() << std::endl;
					}	
				}
				else	startPoint = 0.5*(lowerBound + upperBound);                 // No solution yet; start in the middle
				
				try // To solve the QP problem
				{
					dq = solve(H,f,
					           this->B.block(0,12,10+2*this->numJoints,this->numJoints), // Remove component for Lagrange multipliers	
						   this->z, startPoint);
				}
				catch(const std::exception &exception)
				{
					std::cout << exception.what() << std::endl;
					dq.setZero();                                               // Couldn't solve control problem; don't move
				}
			}
			else
			{
				// H = [ 0  J ]
				//     [ J' M ]
				H.resize(12+this->numJoints,12+this->numJoints);
				H.block( 0, 0,              12,              12).setZero();
				H.block( 0,12,              12, this->numJoints) = this->J;
				H.block(12, 0, this->numJoints,              12) = this->J.transpose();
				H.block(12,12, this->numJoints, this->numJoints) = this->M;
				
				// f = [        -dx        ]
				//     [  -M*redundantTask ]
				f.resize(12+this->numJoints);
				f.head(12)              = -dx;
				f.tail(this->numJoints) = -M*redundantTask;
				
				// Get the start point for the QP solver
				startPoint.resize(12+this->numJoints);                              // +12 for Lagrange multipliers
				startPoint.head(12) = (this->J*this->invM*this->J.transpose()).partialPivLu().solve(this->J*redundantTask - dx); // Lagrange multipliers
				if(not last_solution_exists())
				{
					startPoint.tail(this->numJoints) = 0.5*(lowerBound + upperBound);   // Start in the middle of the boundaries
				}
				else
				{
					try // to get the last solution, if it exists
					{
						startPoint.tail(this->numJoints) = last_solution().tail(this->numJoints); // Try to get the previous solution
						
						// Ensure it's within the new bounds
						for(int i = 0; i < this->numJoints; i++)
						{
							     if(startPoint(12+i) < lowerBound(i)) startPoint(12+i) += 0.001;
							else if(startPoint(12+i) > upperBound(i)) startPoint(12+i) -= 0.001;
						}
					}
					catch(const std::exception &exception)
					{
						std::cout << exception.what() << std::endl;         // Something went wrong somehow
					}
					
					try // to solve the QP problem
					{
						dq = (solve(H,f,this->B,this->z,startPoint)).tail(this->numJoints); // We don't need the Lagrange multipliers for the solution
					}
					catch(const std::exception &exception)
					{
						std::cout << exception.what() << std::endl;         // Inform user
						dq.setZero();                                       // Don't move
					}
				}
			}
			
			// Re-solve the problem subject to grasp constraints Jc*qdot = 0
			if(this->is_grasping())
			{
				std::cout << "Worker bees can leave.\n";
				std::cout << "Even drones can fly away.\n";
				std::cout << "The Queen is their slave.\n";
			}
			
			this->qRef += dq;                                                           // Update the reference point
		}
		
		if(not send_joint_commands(qRef)) std::cerr << "[ERROR] [ICUB 2] Could not send joint commands for some reason.\n";
	}
	else
	{
		halt();                                                                             // Stop the robot moving
		close();                                                                            // Close the connection with the joints to prevent problems
	}
}


/* OLD:
	update_state();                                                                             // Update kinematics, dynamics, constraints
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since start of control loop
	
	if(this->controlSpace == joint)
	{
		Eigen::VectorXd qd(this->numJoints);
		Eigen::VectorXd q0(this->numJoints);
		
		for(int i = 0; i < this->numJoints; i++)
		{
			qd(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime);                // Desired state for the given time
			
			double lower = this->positionLimit[i][0];
			double upper = this->positionLimit[i][1];

			this->z(i)                 = -upper;
			this->z(i+this->numJoints) =  lower;
			
			q0(i) = 0.5*(lower + upper);
		}
		
		this->z.tail(10) = -this->b;
		
		try // to solve the QP problem
		{
			qRef = solve(Eigen::MatrixXd::Identity(this->numJoints,this->numJoints),    // H
				    -qd,                                                            // f
				     this->B.block(0,12,10+2*this->numJoints,this->numJoints),      // Remove component for Lagrange multipliers in Cartesian mode
				     this->z,                             
				     q0);
		}
		catch(const char* error_message)
		{
			std::cerr << error_message << std::endl;
		}
	}
	else
	{
		Eigen::VectorXd dx = track_cartesian_trajectory(elapsedTime);                       // Feedforward + feedback control
		Eigen::VectorXd redundantTask = 0.01*(this->setPoint - this->q);                    // As it says on the label
		Eigen::VectorXd q0 = Eigen::VectorXd::Zero(this->numJoints);
		
		// Solve for instantaneous joint limits
		for(int i = 0; i < this->numJoints; i++)
		{
			double lower, upper;
			compute_joint_limits(lower,upper,i);
			this->z(i) = -upper;
			this->z(i+this->numJoints) = lower;
			q0(i) = 0.5*(lower + upper);
		}
		this->z.tail(10) = -(this->A*this->qRef + this->b);
		
		// H = [ 0  J ]
		//     [ J' M ]
		Eigen::MatrixXd H(12+this->numJoints,12+this->numJoints);
		H.block( 0, 0,             12,             12).setZero();
		H.block( 0,12,             12,this->numJoints) = this->J;
		H.block(12, 0,this->numJoints,             12) = this->J.transpose();
		H.block(12,12,this->numJoints,this->numJoints) = this->M;
		
		// f = [      -dx       ]
		//   = [ -M*redundantTask ]
		Eigen::VectorXd f(12+this->numJoints);
		f.head(12)              = -dx;                                                      // Primary task
		f.tail(this->numJoints) = -M*redundantTask;
		
		// Compute the start point for the QP solver
		Eigen::VectorXd startPoint(12+this->numJoints);
		startPoint.head(12) = (this->J*this->Minv*this->J.transpose()).partialPivLu().solve(this->J*redundantTask - dx); // These are the Lagrange multipliers
		startPoint.tail(this->numJoints) = q0;
		
		Eigen::VectorXd dq(this->numJoints);
		
		try
		{
			dq = (solve(H,f,this->B,z,startPoint)).tail(this->numJoints);               // We can ignore the Lagrange multipliers
		}
		catch(const char* error_message)
		{
			std::cerr << error_message << std::endl;                                    // Inform user
			dq.setZero();                                                               // Don't move
		}
		
		// Re-solve the problem subject to grasp contraints Jc*qdot = 0
		if(this->isGrasping)
		{	
			Eigen::MatrixXd Jc = this->C*this->J;                                       // Constraint Jacobian
			
			// Set up the new start point for the solver
			startPoint.resize(6+this->numJoints);
			startPoint.head(6)  = (Jc*this->Minv*Jc.transpose())*Jc*dq;                 // Lagrange multipliers
			startPoint.tail(12) = dq;                                                   // Previous solution
			
			// H = [ 0   Jc ]
			//     [ Jc' M  ]
			H.resize(6+this->numJoints,6+this->numJoints);
			H.block(0,0,              6,              6).setZero();
			H.block(0,6,              6,this->numJoints) = Jc;
			H.block(6,0,this->numJoints,              6) = Jc.transpose();
			H.block(6,6,this->numJoints,this->numJoints) = M;
			
			// f = [   0   ]
			//     [ -M*dq ]
			f.resize(6+this->numJoints);
			f.head(6).setZero();
			f.tail(this->numJoints) = -this->M*dq;
			
			try // to solve the constrained motion
			{
				dq = (solve(H,
				            f,
				            this->B.block(0,6,10+2*this->numJoints,6+this->numJoints), // Reduce the constraint since we now have -6 Lagrange multipliers
				            z,
				            startPoint)
				     ).tail(this->numJoints);                                       // We can throw away the Lagrange multipliers from the result
			}
			catch(const char* error_message)
			{
				std::cerr << error_message << std::endl;                            // Inform user
				dq.setZero();                                                       // Don't move
			}
		}
		
		this->qRef += dq;                                                                   // Update the reference position
	}

//	for(int i = 0; i < this->numJoints; i++) send_joint_command(i,qRef[i]);
*/
