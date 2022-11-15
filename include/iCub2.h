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
	
	// Now generate the larger constraint for the QP solver:
	//      B*qdot + z >= 0
	// where:
	//      B = [  -I  ]    z = [  -vMax  ]
	//          [   I  ]        [   vMin  ]
	//          [ dt*A ]        [ A*q + b ]
	
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
	// Variables used in this scope:
	Eigen::VectorXd vel = Eigen::VectorXd::Zero(this->n);                                       // We want to compute this

	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since start of control loop
	
	if(this->controlMode == joint)
	{
		// We need these to get the desired state from the trajectory generator
		iDynTree::VectorDynSize     q_d(this->n),                                           // Desired position
		                         qdot_d(this->n),                                           // Desired velocity
		                        qddot_d(this->n);                                           // Desired acceleration (not used for iCub2)
	
		Eigen::VectorXd ref(this->n);                                                       // Desired + error feedback
		Eigen::VectorXd initialGuess(this->n);                                              // Needed for the QP solver
		for(int i = 0; i < this->n; i++)
		{
			ref[i] = qdot_d[i] + this->kq*(q_d[i] + this->q[i]);                        // Solve the feedforward + feedback control

			double minSpeed, maxSpeed;
			get_speed_limits(minSpeed,maxSpeed,i);                                      // Get the instantaneous speed limits
			
			
			initialGuess[i] = 0.5*(minSpeed + maxSpeed);                                // Halfway
			
			z[i]         = -maxSpeed;
			z[i+this->n] =  minSpeed;
			
			// Note to self: Putting joint limits here is kind of superflous because
			// the trajectory generator ensures limits are obeyed, but it makes the code
			// neater when solving the Cartesian control.
		}
		
		z.tail(10) = this->A*this->q + this->b;                                             // For shoulder limits
		
		std::cout << z << std::endl;
		
		
		// Solve QP problem of the form:
		//     min  0.5*x'*H*x - x'*f
		//     subject to: B*x > z
		Eigen::VectorXd vel = solve(Eigen::MatrixXd::Identity(this->n,this->n),             // H
		                           -ref,                                                    // f
		                            this->B,                                                
		                            this->z,
		                            initialGuess);
	}
	else if(this->controlMode == cartesian)
	{
		// Even drones can fly away.
	}
	else if(this->controlMode == grasp)
	{
		// The Queen is their slave.
	}
	
	for(int i = 0; i < this->n; i++) send_velocity_command(vel[i],i);                           // Send commands to the respective joint motors
}
