    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //        A class that defines the dynamic properties of an object being carried by a robot       //   
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PAYLOAD_H_
#define PAYLOAD_H_

#include <Eigen/Geometry>                                                                           // Eigen::Isometry3d
#include <iostream>                                                                                 // std::cout, std::cerr

class Payload
{
	public:
	
		// Constructors
		Payload() {}                                                                        // Empty constructor
		
		Payload(const double &_mass,
		        const Eigen::Matrix<double,3,3> &_inertia);                                 
		
		Payload(const double &_mass,
		        const double &Ixx,
		        const double &Ixy,
		        const double &Ixz,
		        const double &Iyy,
		        const double &Iyz,
		        const double &Izz) :
		Payload(_mass,
		       (Eigen::MatrixXd(3,3) << Ixx, Ixy, Ixz,
		                                Ixy, Iyy, Iyz,
		                                Ixz, Iyz, Izz).finished()) {}
		                                     
		        
		Eigen::Isometry3d pose() const { return this->globalPose; }                         // Get the pose in the global coordinates
		
		void update_state(const Eigen::Isometry3d &globalToLocal,
		                  const Eigen::Matrix<double,6,1> &velocity);
		
	private:

		double mass;
		
		Eigen::Isometry3d localPose;                                                        
		
		Eigen::Isometry3d globalPose;                                               
		
		Eigen::Matrix<double,3,3> localInertia;
		
		Eigen::Matrix<double,3,3> globalInertia;
		
		Eigen::Matrix<double,3,3> inertiaDerivative;
		
		Eigen::Matrix<double,3,1> twist;
		
		
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        CONSTRUCTOR                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Payload::Payload(const double &_mass,
                 const Eigen::Matrix<double,3,3> &inertia) :
                 mass(_mass),
                 localInertia(inertia)
{
	// Check that the matrix is symmetric
	if( (inertia - inertia.transpose()).norm() > 1e-03 )
	{
		std::cerr << "[ERROR] [PAYLOAD] Constructor(): "
		          << "The inertia matrix is not symmetric!" << std::endl;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Set new state, update dynamics                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void Payload::update_state(const Eigen::Isometry3d &globalToLocal,
                           const Eigen::Matrix<double,6,1> &velocity) // NOTE: this is the velocity at the contact point
{
	this->globalPose = globalToLocal * this->localPose;                                         // Update pose
	
	// Velocity argument is for the external contact point,
	// so we need to transform it to the center of the payload
	
	Eigen::Vector3d r = this->globalPose.translation() - globalToLocal.translation();           // Translation to center from contact point
	Eigen::Vector3d w = velocity.tail(3);                                                       // Angular velocity component
	
	this->twist.head(3) = velocity.head(3) + w.cross(r);
	this->twist.tail(3) = w;
	
	// Rotate the inertia matrix to the new coordinates
	Eigen::Matrix<double,3,3> R = this->globalPose.rotation();
	this->globalInertia = R*this->localInertia*R.transpose();
	
	// Update the time derivative of the inertia matrix
	// Idot = skew(w)*I, where w is the angular velocity.
	// We can skip the zeros along the diagonal by solving manually.

	this->inertiaDerivative << this->twist(4)*this->globalInertia(2,0) - this->twist(5)*this->globalInertia(1,0),
	                           this->twist(4)*this->globalInertia(2,1) - this->twist(5)*this->globalInertia(1,1),
	                           this->twist(4)*this->globalInertia(2,2) - this->twist(5)*this->globalInertia(1,2),
	                           
	                           this->twist(5)*this->globalInertia(0,0) - this->twist(3)*this->globalInertia(2,0),
	                           this->twist(5)*this->globalInertia(0,1) - this->twist(3)*this->globalInertia(2,1),
	                           this->twist(5)*this->globalInertia(0,2) - this->twist(3)*this->globalInertia(2,2),
	                           
	                           this->twist(3)*this->globalInertia(1,0) - this->twist(4)*this->globalInertia(0,0),
	                           this->twist(3)*this->globalInertia(1,1) - this->twist(4)*this->globalInertia(0,1),
	                           this->twist(3)*this->globalInertia(1,2) - this->twist(4)*this->globalInertia(0,2);
}

#endif
