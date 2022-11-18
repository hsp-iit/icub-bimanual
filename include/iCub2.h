  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Custom class for 2-handed control of iCub 2                          //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iCubBase.h>                                                                               // Custom class: most functions defined here
		        
class iCub2 : public iCubBase
{
	public:
		iCub2(const std::string &fileName,
		      const std::vector<std::string> &jointList,
		      const std::vector<std::string> &portList);
	
	private:
		// Shoulder constraints
		Eigen::Matrix<double,10,17> A;
		Eigen::Matrix<double,10,1>  b;
		
		// Inequality constraints on the QP solver
		Eigen::Matrix<double,44,17> B;
		Eigen::Matrix<double,44,1>  z;
		
		void run();                                                                         // Main control loop
			
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Constructor for the iCub 2                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCub2::iCub2(const std::string &fileName,
             const std::vector<std::string> &jointList,
             const std::vector<std::string> &portList) :
             iCubBase(fileName, jointList, portList)
{
	// Lower the gains since we're running in velocity mode
	set_joint_gains(5.0, 0.01);                                                                // We don't actually care about the derivative
	
	// Set the constraints for the iCub2 shoulder tendons.
	// A *single* arm is constrained by
	//      A*q + b > 0,
	// but we have two arms so we need to double up the constraint matrix.
	
	double c = 1.71;
	this->A.setZero();
	this->A.block(0,3,5,3) <<  c, -c,  0,
	                           c, -c, -c,
	                           0,  1,  1,
	                          -c,  c,  c,
	                           0, -1, -1;
	                           
	this->A.block(5,10,5,3) = this->A.block(0,3,5,3);                                           // Same constraint for right arm as left arm
	
	this->A *= 180/M_PI;                                                                        // Convert from rad to deg
	
	this->b.head(5) << 347.00,
	                   366.57,
	                    66.60,
	                   112.42,
	                   213.30;
	                   
	this->b.tail(5) = this->b.head(5);                                                          // Same constraint for the right arm as the left arm
	
	// Now generate the larger constraint for the QP solver:
	//      B*qdot >= z
	// where:
	//      B = [  -I  ]    z = [   -vMax    ]
	//          [   I  ]        [    vMin    ]
	//          [ dt*A ]        [ -(A*q + b) ]
	
	B.block( 0,0,17,17) = -1*Eigen::MatrixXd::Identity(17,17);
	B.block(17,0,17,17) =    Eigen::MatrixXd::Identity(17,17);
	B.block(34,0,10,17) =    this->dt*A;
	
	// Note: z is dynamic, so there's no point setting it here
	
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCub2::run()
{
	update_state();                                                                             // Read joints, update kinematics
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Compute time since start of control thread
	
	Eigen::VectorXd vel = Eigen::VectorXd::Zero(this->n);                                       // We want to compute this
	
	if(this->controlMode == joint)
	{
		double q_d, qdot_d, qddot_d;                                                        // Desired state of a single joint
		Eigen::VectorXd ref(this->n);                                                       // Desired + error feedback
		Eigen::VectorXd initialGuess(this->n);                                              // Needed for the QP solver
		
		for(int i = 0; i < this->n; i++)
		{
			q_d = this->jointTrajectory[i].evaluatePoint(elapsedTime, qdot_d, qddot_d); // Compute the desired state for given time
			
			ref[i] = qdot_d + this->kq*(q_d - this->q[i]);                              // Feedforward + feedback control

			/*
			double minSpeed, maxSpeed;
			get_speed_limits(minSpeed,maxSpeed,i);                                      // Get the instantaneous speed limits
			
			initialGuess[i] = 0.5*(minSpeed + maxSpeed);                                // Halfway
			
			z[i]         = -maxSpeed;
			z[i+this->n] =  minSpeed;
			
			// NOTE: Putting joint limits here is kind of superflous because the
			//       "move_to_position()" function ensures it is always within position
			//       limits, but it means we can use the same constraints for Cartesian
                        //       control.
                 	*/
		}
		
		/*
		// NOTE: The iCub wiki specifies the constraint as Aq + b > 0,
		//       but for the QP solver we need B*qdot >= z
		z.tail(10) = -(this->A*this->q + this->b);                                          // For shoulder limits
		
		// Solve the problem:
		//      min 0.5*qdot'H*qdot - qdot'*ref
		//      subject to:  B*qdot >= z
		vel = solve(Eigen::MatrixXd::Identity(this->n,this->n),                             // H
		           -ref,                                                                    // f
		            this->B,                                                
		            this->z,
		            initialGuess);
		*/
		vel = ref;
	}
	else
	{
		// Generate the Jacobian
		Eigen::MatrixXd J(12,this->n);                                                      // Jacobian for both hands
		Eigen::MatrixXd subJ(6,6+this->n);                                                  // Jacobian for a single hand
		
		this->computer.getFrameFreeFloatingJacobian("left", subJ);                          // Get the full left hand Jacobian
		J.block(0,0,6,this->n) = subJ.block(0,6,6,this->n);                                // Assign to the larger Jacobian
		if(not this->leftControl) J.block(0,0,6,3).setZero();                               // Remove contribution of torso joints
		
		this->computer.getFrameFreeFloatingJacobian("right", subJ);                         // Get the full right hand Jacobian
		J.block(6,0,6,this->n) = subJ.block(0,6,6,this->n);                                 // Assign the right hand Jacobian
		if(not this->rightControl) J.block(6,0,6,3).setZero();                              // Remove contribution of torso joints
		
		Eigen::MatrixXd Jt = J.transpose();                                                 // Makes calcs a little easier later
		
		// Compute the joint inertia matrix, add joint limit avoidance
		Eigen::MatrixXd M(6+this->n,6+this->n);                                             // Storage location
		this->computer.getFreeFloatingMassMatrix(M);                                        // Inertia including floating base
		M = M.block(6,6,this->n,this->n);                                                   // Remove the floating base part
		
		for(int i = 0; i < this->n; i++) M(i,i) += get_joint_penalty(i) - 1;                // Add penalty for joint limit avoidance
		
		// Compute the relevant hand velocity
		Eigen::Matrix<double,12,1> xdot; xdot.setZero();                                    // Left hand and right hand
		yarp::sig::Matrix pose(4,4);                                                        // Desired pose for a hand
		yarp::sig::Vector poseError(6);                                                     // As it says on the label
		yarp::sig::Vector vel(6), acc(6);                                                   // Desired velocity and acceleration
		
		if(this->leftControl)
		{
			this->leftTrajectory.get_state(pose,vel,acc,elapsedTime);                   // Desired state for the given time
			
			poseError = get_pose_error(pose, convert_iDynTree_to_yarp(this->computer.getWorldTransform("left")));
			
			for(int i = 0; i < 6; i++)
			{
				xdot(i) = vel(i);                                                   // Feedforward term
				for(int j = 0; j < 6; j++) xdot(i) += this->K(i,j)*poseError(j);    // Feedback term
			}
		}
		
		if(this->rightControl)
		{
			this->rightTrajectory.get_state(pose,vel,acc,elapsedTime);
			poseError = get_pose_error(pose, convert_iDynTree_to_yarp(this->computer.getWorldTransform("right")));
			
			for(int i = 0; i < 6; i++)
			{
				xdot(i+6) = vel(i);
				for(int j = 0; j < 6; j++) xdot(i+6) += this->K(i,j)*poseError(j);
			}
		}
		
	}
	
	for(int i = 0; i < this->n; i++) send_velocity_command(vel[i],i);                           // Send commands to the respective joint motors
}
