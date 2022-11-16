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
			
			std::cout << "Joint " << i+1 << " tracking error: " << (q_d - this->q[i])*180/M_PI << std::endl;

/*			double minSpeed, maxSpeed;
			get_speed_limits(minSpeed,maxSpeed,i);                                      // Get the instantaneous speed limits
			
			initialGuess[i] = 0.5*(minSpeed + maxSpeed);                                // Halfway
			
			z[i]         = -maxSpeed;
			z[i+this->n] =  minSpeed;
			
			// NOTE: Putting joint limits here is kind of superflous because the
			//       "move_to_position()" function ensures it is always within position
			//       limits, but it means we can use the same constraints for Cartesian
                        //       control.
		}
		
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
		            initialGuess);*/
		 }           
		
		vel = ref;
	}
	else std::cout << "Not yet programmed you fool!" << std::endl;
	
	for(int i = 0; i < this->n; i++) send_velocity_command(vel[i],i);                           // Send commands to the respective joint motors
}
