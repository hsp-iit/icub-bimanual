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
		Eigen::MatrixXd A;
		Eigen::Matrix<double,10,1>  b;
		
		// General constraint matrices for QP solver
		Eigen::MatrixXd constraintMatrix;
		Eigen::VectorXd constraintVector;
		
		Eigen::VectorXd setPoint; // Desired joint configuration
		
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
	set_cartesian_gains(10.0, 0.01);
	
	// Set the constraints for the iCub2 shoulder tendons.
	// A *single* arm is constrained by
	//      A*q + b > 0,
	// but we have two arms so we need to double up the constraint matrix.
	
	double c = 1.71;
	this->A = Eigen::MatrixXd::Zero(10,this->n);
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
	// dt*A*qdot >= -(A*q + b)
	
	// The constraint matrix is unique to iCub2.
	// constraintMatrix = [ 0   -I ]
	//                    [ 0    I ]
	//                    [ 0 dt*A ]
	this->constraintMatrix.resize(10+2*this->n,12+this->n);                                     // 2*n for joint limits, 10 for shoulder limits
	this->constraintMatrix.block(        0, 0,10+2*this->n,12)      = Eigen::MatrixXd::Zero(10+2*this->n,12);
	this->constraintMatrix.block(        0,12,     this->n,this->n) =-Eigen::MatrixXd::Identity(this->n,this->n);
	this->constraintMatrix.block(  this->n,12,     this->n,this->n) = Eigen::MatrixXd::Identity(this->n,this->n);
	this->constraintMatrix.block(2*this->n,12,          10,this->n) = this->dt*this->A;
	
	// constraintVector = [ -qdot_max ]
	//           [  qdot_min ]
	//           [-(A*q + b) ]
	this->constraintVector.resize(10+2*this->n);                                                // Vector is dynamic, so no need to set it here
	
	// Set the desired configuration for the arms when running in Cartesian mode
	this->setPoint.resize(this->n);
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
	update_state();                                                                             // Update kinematics, dynamics, constraints
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since start of control loop
	
	Eigen::VectorXd vel = Eigen::VectorXd::Zero(this->n);                                       // We want to compute this
	
	this->constraintVector.head(2*this->n) = this->z;                    			    // Upper and lower limits on joints
	this->constraintVector.tail(10)        =-(this->A*this->q + this->b);                       // Shoulder constraints for iCub2
		
	if(this->controlSpace == joint)
	{	
		Eigen::VectorXd ref = track_joint_trajectory(elapsedTime);                          // Get the reference velocity
		
		// min 0.5*(qdot_ref - qdot)'*(qdot_ref - qdot)
		// subject to: B*qdot > z
		vel = solve(Eigen::MatrixXd::Identity(this->n,this->n),                             // H
		           -ref,                                                                    // f
		            this->constraintMatrix.block(0,12,10+2*this->n,this->n)                 // B
		            this->constraintVector,                                                 // z
		            this->x0);                                                              // Initial guess for solver
	}
	else if(this->controlSpace == cartesian)
	{
		Eigen::VectorXd ref = track_cartesian_trajectory(elapsedTime);                      // Get the Cartesian velocity vector
		
		// min (qdot_0 - qdot)'*M*(qdot_0 - qdot)
		// subject to: J*qdot = x
		//             B*qdot > z
		
		vel = solve(this->H,
		            this->f,
		            this->constraintMatrix,                                                 // Unique to iCub2
		            this->constraintVector,                                                 // Unique to iCub2
			    initialGuess);
			    
		
	}
	else
	{
		std::cerr << "[ERROR] [iCUB2] run(): "
		          << "Control space incorrectly specified! How did that happen?" << std::endl;
	}

/*
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
			// Compute the feedforward + feedback control for the ith joint
			q_d = this->jointTrajectory[i].evaluatePoint(elapsedTime, qdot_d, qddot_d); // Desired state for given time
			
			ref[i] = qdot_d + this->kq*(q_d - this->q[i]);                              // Feedforward + feedback control
			
			// Solve for initial guess and instantaneous joint limits
			double minSpeed, maxSpeed;
			get_speed_limits(minSpeed, maxSpeed, i);
			
			this->z(i)         =-maxSpeed;
			this->z(i+this->n) = minSpeed;
			
			initialGuess(i) = 0.5*(minSpeed + maxSpeed);
		}
		this->z.tail(10) = -(this->A*this->q + this->b);

		vel = solve(Eigen::MatrixXd::Identity(this->n,this->n),                             // H
			    -ref,                                                                   // f
			     this->Bsub,                                                            // "B" without lagrange multipliers
			     this->z,                                                               // z
			     initialGuess);                                                         // Starting point for solver
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
		
		Eigen::MatrixXd W = M.block(6,6,this->n,this->n);                                   // Remove the floating base part
		for(int i = 0; i < this->n; i++) W(i,i) += get_joint_penalty(i) - 1;                // Add penalty for joint limit avoidance
		
		// Compute the relevant hand velocity
		Eigen::Matrix<double,12,1> xdot; xdot.setZero();                                    // Left hand and right hand
		yarp::sig::Matrix pose(4,4);                                                        // Desired pose for a hand
		yarp::sig::Vector poseError(6);                                                     // As it says on the label
		yarp::sig::Vector v(6), a(6);                                                       // Desired velocity and acceleration
		iDynTree::Transform T;                                                              // Temporary storage
		
		// NOTE TO SELF: There's probably a neater way to do this
		if(this->leftControl)
		{
			this->leftTrajectory.get_state(pose,v,a,elapsedTime);                       // Get desired state
			
			T = this->computer.getWorldTransform("left");                               // Current hand pose
			
			poseError = get_pose_error(pose,convert_iDynTree_to_yarp(T));               // Pose eror
			
			for(int i = 0; i < 6; i++) xdot[i] = v[i] + this->K(i,i)*poseError[i];     // Feedback control
		}

		if(this->rightControl)
		{
			this->rightTrajectory.get_state(pose,v,a,elapsedTime);                   
			
			T = this->computer.getWorldTransform("right");
			
			poseError = get_pose_error(pose,convert_iDynTree_to_yarp(T));
			
			for(int i = 0; i < 6; i++) xdot[i+6] = v[i] + this->K(i,i)*poseError[i];
		}
		
		// Variables needed for the QP solver
		Eigen::MatrixXd H(12+this->n,12+this->n);
		Eigen::VectorXd f(12+this->n);
		Eigen::VectorXd initialGuess(12+this->n);
		
		// H = [ 0   J ]
		//     [ J'  M ]
		H.block( 0, 0,     12,     12) = Eigen::MatrixXd::Zero(12,12);
		H.block( 0,12,     12,this->n) = J;
		H.block(12, 0,this->n,     12) = Jt;
		H.block(12,12,this->n,this->n) = M;
		
		// f = [   xdot    ]
		//     [ redundant ]
		f.head(12)      = xdot;                                                        
		f.tail(this->n) = 0.1*(this->setPoint - this->q);


		initialGuess.head(12) = (J*M.inverse()*Jt).partialPivLu().solve(-xdot);             // Lagrange multipliers
		for(int i = 0; i < this->n; i++)		
		{
			double minSpeed, maxSpeed;
			get_speed_limits(minSpeed,maxSpeed,i);
			this->z(i)         =-maxSpeed;
			this->z(i+this->n) = minSpeed;
			
			initialGuess(12+i) = 0.5*(minSpeed + maxSpeed);                             // Halfway between limits for QP solver
		}
		this->z.tail(10) = -(this->A*this->q + this->b);
		
		// NOTE TO SELF: Be sure to NEGATE f here, otherwise it will diverge
		vel = (solve(H,-f,this->B,this->z,initialGuess)).tail(this->n);                     // We don't care about the Lagrange multipliers
	}
	
//	std::cout << "Distance to constraint:" << std::endl;
//	std::cout << this->A*this->q + this->b << std::endl;
	
	for(int i = 0; i < this->n; i++) send_velocity_command(vel[i],i);                           // Send commands to the respective joint motors
*/
}
