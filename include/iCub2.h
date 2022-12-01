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
	
	// Update the constraints
	this->constraintVector.tail(10) = -(this->A*this->q + this->b);                             // Shoulder constraints
	
	if(this->controlSpace == joint)
	{
		Eigen::VectorXd ref = track_joint_trajectory(elapsedTime);                          // Solve feedforward/feedback control
		
		vel = solve(Eigen::MatrixXd::Identity(this->n,this->n),                             // H
		           -ref,                                                                    // f
		            this->constraintMatrix.block(0,12,10+2*this->n,this->n),                // B; remove part related to Lagrange multipliers
		            this->constraintVector,                                                 // z
		            initialGuess);
	}
	else
	{
		Eigen::VectorXd xdot = track_cartesian_trajectory(elapsedTime);                     // Solve Cartesian feedforward / feedback control
		
		Eigen::VectorXd redundantTask = 2*(this->setPoint - this->qdot);                    
		
		Eigen::VectorXd f(12+this->n);
		f.head(12)      = -xdot;
		f.tail(this->n) = -this->M*redundantTask();
		
		Eigen::VectorXd initialGuess(12+this->n);
		initialGuess.head(12) = (this->J*this->M.inverse()*this->J.transpose()).partialPivLu().solve(this->J*redundantTask - xdot);
		initialGuess.tail(this->n) = this->qpStartPoint;
		
		vel = solve(this->H,f,this->constraintMatrix,this->constraintVector,initialGuess);	
	}
}
