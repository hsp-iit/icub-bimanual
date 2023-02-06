  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Custom class for 2-handed control of iCub 2                          //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ICUB2_H_
#define ICUB2_H_

#include <Eigen/Geometry>                                                                               // Eigen::Isometry3d and the like
#include <iCubVelocity.h>                                                                               // Custom class: most functions defined here
		        
class iCub2 : public iCubVelocity
{
	public:
		iCub2(const std::string &pathToURDF,
		      const std::vector<std::string> &jointNames,
		      const std::vector<std::string> &portNames);
	
	private:
		// Shoulder constraints
		Eigen::MatrixXd A;
		Eigen::Matrix<double,10,1> b;
		
		// General constraint matrices for QP solver
		Eigen::MatrixXd B;
		Eigen::VectorXd z;
		
		Eigen::VectorXd setPoint;                                                           // Desired joint configuration
		
		void run();                                                                         // Main control loop
			
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Constructor for the iCub 2                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCub2::iCub2(const std::string &pathToURDF,
             const std::vector<std::string> &jointNames,
             const std::vector<std::string> &portNames) :
             iCubVelocity(pathToURDF,
                          jointNames,
                          portNames,
                          Eigen::Isometry3d(Eigen::Translation3d(0.0,0.0,0.63)))
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
	// B = [ 0   -I ]
	//     [ 0    I ]
	//     [ 0 dt*A ]
	this->B.resize(10+2*this->n,12+this->n);                                                    // 2*n for joint limits, 10 for shoulder limits
	this->B.block(        0, 0,10+2*this->n,12)      = Eigen::MatrixXd::Zero(10+2*this->n,12);
	this->B.block(        0,12,     this->n,this->n) =-Eigen::MatrixXd::Identity(this->n,this->n);
	this->B.block(  this->n,12,     this->n,this->n) = Eigen::MatrixXd::Identity(this->n,this->n);
	this->B.block(2*this->n,12,          10,this->n) = this->dt*this->A;
	
	this->z.resize(10+2*this->n);
	
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
	
	Eigen::VectorXd upperBound(this->n), lowerBound(this->n);
	
	// Compute the constraint vector and start point
	for(int i = 0; i < this->n; i++) compute_speed_limits(lowerBound(i),upperBound(i),i);
	this->z.head(this->n)              = -upperBound;
	this->z.block(this->n,0,this->n,1) = lowerBound;
	this->z.tail(10)                   = -(this->A*this->q + b);
	
	if(this->controlSpace == joint)
	{
		Eigen::VectorXd desiredVel = track_joint_trajectory(elapsedTime);                   // As it says on the label              
		
		// We need to solve a unique constrained problem for the iCub 2
		vel = solve(Eigen::MatrixXd::Identity(this->n,this->n),                             // H
		           -desiredVel,                                                             // f
		            this->B.block(0,12,10+2*this->n,this->n),                               // B (without Lagrange multipliers)
		            z,                                                                      // z
		            0.5*(lowerBound + upperBound));                                         // x0
	}
	else
	{	
		Eigen::VectorXd xdot = track_cartesian_trajectory(elapsedTime);                     // Feedforward + feedback control
		Eigen::VectorXd redundantTask = -0.5*(this->setPoint - this->q);                    // As it says on the label
		
		// H = [ 0  J ]
		//     [ J' M ]
		Eigen::MatrixXd H(12+this->n,12+this->n);
		H.block( 0, 0,     12,     12).setZero();
		H.block( 0,12,     12,this->n) = this->J;
		H.block(12, 0,this->n,     12) = this->J.transpose();
		H.block(12,12,this->n,this->n) = this->M;
		
		// f = [      -xdot       ]
		//   = [ -M*redundantTask ]
		Eigen::VectorXd f(12+this->n);
		f.head(12)      = -xdot;                                                            // Primary task
		f.tail(this->n) = -M*redundantTask;
		
		// Compute the start point for the QP solver
		Eigen::VectorXd startPoint(12+this->n);
		startPoint.head(12) = (this->J*this->M.partialPivLu().inverse()*this->J.transpose()).partialPivLu().solve(
		                       this->J*redundantTask - xdot);                               // These are the Lagrange multipliers
		                       
		startPoint.tail(this->n) = 0.5*(lowerBound + upperBound);                           // These are the joint velocities
		
		vel = (solve(H,f,this->B,z,startPoint)).tail(this->n);                              // We can ignore the Lagrange multipliers
	}
	
	for(int i = 0; i < this->n; i++) send_velocity_command(vel(i),i);
}

#endif
