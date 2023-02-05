    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //               Specification of torque control functions for the iCub/ergoCub                //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ICUB_TORQUE_H_
#define ICUB_TORQUE_H_

#include <iCubBase.h>

class iCubTorque : public iCubBase
{
	public:
		iCubTorque(const std::string &pathToURDF,
			     const std::vector<std::string> &jointNames,
			     const std::vector<std::string> &portNames,
			     const Eigen::Isometry3d &_torsoPose) :
			     iCubBase(pathToURDF,
			              jointNames,
			              portNames,
			              _torsoPose) {}
			              
		// Inherited from the iCubBase class
		void compute_speed_limits(double &lower, double &upper, const int &i);
			     
		Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time);
		
		Eigen::VectorXd track_joint_trajectory(const double &time);
		
		// Inherited from yarp::PeriodicThread class
		void threadRelease();
			
};                                                                                                  // Semicolon needed after class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Solve velocity to track Cartesian trajectory                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,12,1> iCubTorque::track_cartesian_trajectory(const double &time)
{
	// Variables used in this scope
	Eigen::Matrix<double,12,1> xddot; xddot.setZero();
	Eigen::Isometry3d pose;
	Eigen::Matrix<double,6,1> vel, acc;
	
	if(this->leftControl)
	{
		this->leftTrajectory.get_state(pose, vel, acc, time);
		
		xddot.head(6) = acc
		              + this->D*(vel - this->J.block(0,0,6,this->n)*this->qdot)
		              + this->K*pose_error(pose, iDynTree_to_Eigen(this->computer.getWorldTransform("left")));
	}
	else    xddot.head(6) = -this->D*this->J.block(0,0,6,this->n)*this->qdot;
	
	if(this->rightControl)
	{
		this->rightTrajectory.get_state(pose, vel, acc, time);
		
		xddot.tail(6) = acc
		              + this->D*(vel - this->J.block(6,0,6,this->n)*this->qdot)
		              + this->K*pose_error(pose, iDynTree_to_Eigen(this->computer.getWorldTransform("right")));
	}
	else	xddot.tail(6) = -this->D*this->J.block(6,0,6,this->n)*this->qdot;
		
	return xddot;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve velocity to track joint trajectory                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd iCubTorque::track_joint_trajectory(const double &time)
{
	Eigen::VectorXd ref(this->n);
	double pos, vel, acc;
	
	for(int i = 0; i < this->n; i++)
	{
		pos = this->jointTrajectory[i].evaluatePoint(time,vel,acc);
		
		ref[i] = acc + this->kd*(vel - this->qdot[i]) + this->kp*(pos - this->q[i]);
	}

	return ref;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute instantaneous velocity limits on the joints                          //
///////////////////////////////////////////////////////////////////////////////////////////////////
void iCubTorque::compute_speed_limits(double &lower, double &upper, const int &i)
{
	lower = std::max( ( this->pLim[i][0] - this->q[i] - this->dt*this->qdot[i])/(this->dt*this->dt),
	        std::max( (-this->vLim[i] - this->qdot[i])/this->dt,
	                   -this->maxAcc));
	                   
	upper = std::min( ( this->pLim[i][1] - this->q[i] - this->dt*this->qdot[i] )/(this->dt*this->dt),
	        std::min( ( this->vLim[i] - this->qdot[i])/this->dt,
	                    this->maxAcc ));
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCubTorque::threadRelease()
{
	std::vector<double> command;
//	iDynTree::FreeFloatingGeneralizedTorques temp; // WTF why doesn't this work?!
//	this->computer.generalizedGravityForces(temp);
//	command = temp.jointTorques();
	
//	send_torque_commands(command); // NOTE: Need to re-write function as send_joint_commands()
}

#endif
