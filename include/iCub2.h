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
		
		void run();                                                                         // Main control loop
			
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Constructor for the iCub 2                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCub2::iCub2(const std::string &fileName,
             const std::vector<std::string> &jointList,
             const std::vector<std::string> &portList):
             iCubBase(fileName,
                      jointList,
                      portList,
                      iDynTree::Transform(iDynTree::Rotation::RPY(0,0,M_PI),
                                          iDynTree::Position(0.00,0.00,0.63)))
{
	// Lower the gains since we're running in velocity mode
	set_joint_gains(5.0, 0.01);                                                                 // We don't actually care about the derivative
	
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
	
	this->b.head(5) << 347.00,
	                   366.57,
	                    66.60,
	                   112.42,
	                   213.30;
	                   
	this->b.tail(5) = this->b.head(5);                                                          // Same constraint for the right arm as the left arm
	
	this->b *= M_PI/180;                                                                        // Convert from degrees to radians
	
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
		}
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
		yarp::sig::Vector v(6), a(6);                                                       // Desired velocity and acceleration
		iDynTree::Transform T;
		
		if(this->leftControl)
		{
			this->leftTrajectory.get_state(pose,v,a,elapsedTime);
			T = this->computer.getWorldTransform("left");
			
			poseError = get_pose_error(pose,convert_iDynTree_to_yarp(T));
			
			for(int i = 0; i < 6; i++) xdot[i] = v[i] + 2.0*poseError[i];
		}

		if(this->rightControl)
		{
			this->rightTrajectory.get_state(pose,v,a,elapsedTime);
			T = this->computer.getWorldTransform("right");
			
			poseError = get_pose_error(pose,convert_iDynTree_to_yarp(T));
			
			for(int i = 0; i < 6; i++) xdot[i+6] = v[i] + 2.0*poseError[i];
		}
		
		// Variables needed for the QP solver
		Eigen::MatrixXd H(12+this->n,12+this->n);
		Eigen::VectorXd f(12+this->n);
		Eigen::MatrixXd B(2*this->n,12+this->n);
		Eigen::VectorXd z(2*this->n);
		Eigen::VectorXd initialGuess(12+this->n);
		
		// H = [ 0   J ]
		//     [ J'  M ]
		H.block( 0, 0,     12,     12) = Eigen::MatrixXd::Zero(12,12);
		H.block( 0,12,     12,this->n) = J;
		H.block(12, 0,this->n,     12) = Jt;
		H.block(12,12,this->n,this->n) = M;
		
		// f = [ -xdot ]
		//     [  0   ]
		f.head(12) = -xdot;
		f.tail(this->n) = Eigen::VectorXd::Zero(this->n);
		
		// B = [ 0  -I ]
		//     [ 0   I ]
		B.block(0,0,2*this->n,12).setZero();
		B.block(0,12,this->n,this->n) = -Eigen::MatrixXd::Identity(this->n,this->n);
		B.block(this->n,12,this->n,this->n) = Eigen::MatrixXd::Identity(this->n,this->n);	
			
		// z = [ -vMax ]
		//     [  vMin ]
		
		initialGuess.head(12) = (J*M.inverse()*Jt).partialPivLu().solve(-xdot);
		for(int i = 0; i < this->n; i++)		
		{
			double minSpeed, maxSpeed;
			get_speed_limits(minSpeed,maxSpeed,i);
			z(i) = -maxSpeed;
			z(i+this->n) = minSpeed;
			
			initialGuess(12+i) = 0.5*(minSpeed + maxSpeed);
		}

		Eigen::VectorXd solution = solve(H,f,B,z,initialGuess);
		
		vel = solution.tail(this->n);
	}
	
	for(int i = 0; i < this->n; i++) send_velocity_command(vel[i],i);                           // Send commands to the respective joint motors
}
