  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Custom class for 2-handed control of iCub 2                          //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ICUB2_H_
#define ICUB2_H_

#include <Eigen/Geometry>                                                                           // Eigen::Isometry3d and the like
#include <PositionControl.h>                                                                        // Custom class: most functions defined here
		        
class iCub2 : public PositionControl
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
             PositionControl(pathToURDF,
                             jointNames,
                             portNames,
                             Eigen::Isometry3d(Eigen::Translation3d(0.0,0.0,0.63)*Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())))
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
	// A*(q + dq) >= b ---> A*dq > -(A*q + b)
	
	// The constraint matrix is unique to iCub2.
	// NOTE: First column here pertains to Lagrange multipliers when running in Cartesian mode
	// B = [ 0   -I ]
	//     [ 0    I ]
	//     [ 0    A ]
	this->B.resize(10+2*this->n,12+this->n);                                                    // 2*n for joint limits, 10 for shoulder limits
	this->B.block(        0, 0,10+2*this->n,12)      = Eigen::MatrixXd::Zero(10+2*this->n,12);
	this->B.block(        0,12,     this->n,this->n) =-Eigen::MatrixXd::Identity(this->n,this->n);
	this->B.block(  this->n,12,     this->n,this->n) = Eigen::MatrixXd::Identity(this->n,this->n);
	this->B.block(2*this->n,12,          10,this->n) = this->A;
	
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

	Eigen::VectorXd dq(this->n); dq.setZero();                                                  // We want to compute this
	
	if(this->controlSpace == joint)
	{
		// Here we solve for dq instead of q directly since it synthesises better with the
		// Cartesian control formulation
		
		
		dq = track_joint_trajectory(elapsedTime);
	}
	
	for(int i = 0; i < this->n; i++) send_joint_command(i,dq(i)+this->q(i));

}

#endif
