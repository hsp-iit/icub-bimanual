    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                         Position control functions for the iCub/ergoCub                       //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POSITION_CONTROL_H_
#define POSITION_CONTROL_H_

#include <iCubBase.h>


class PositionControl : public iCubBase
{
	public:
		PositionControl(const std::string &pathToURDF,
			        const std::vector<std::string> &jointNames,
			        const std::vector<std::string> &portNames,
			        const Eigen::Isometry3d &_torsoPose) :
	        iCubBase(pathToURDF,
	                 jointNames,
	                 portNames,
	                 _torsoPose) {}
		
		// Inherited from the iCubBase class   
		void compute_joint_limits(double &lower, double &upper, const int &i);
			              
		Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time);
		
		Eigen::VectorXd track_joint_trajectory(const double &time);
		
		// Inherited from the yarp::PeriodicThread class
		void threadRelease();
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute instantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::compute_joint_limits(double &lower, double &upper, const int &i)
{
	lower = std::max( this->pLim[i][0] - this->q[i], -this->dt*this->vLim[i] );                 // Maximum between position and velocity
	
	upper = std::min (this->pLim[i][1] - this->q[i],  this->dt*this->vLim[i] );                 // Minimum between position and velocity
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::threadRelease()
{
	for(int i = 0; i < this->n; i++) send_joint_command(i,this->q[i]);                          // Maintain current joint position
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve a discrete time step for Cartesian control                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,12,1> PositionControl::track_cartesian_trajectory(const double &time)
{
	// Variables used in this scope
	Eigen::Matrix<double,12,1> dx; dx.setZero();                                                // Value to be returned
	Eigen::Isometry3d pose;                                                                     // Desired pose
	
	if(this->isGrasping)
	{
		pose = this->payloadTrajectory.get_pose(time);
		
		dx = this->G.transpose() * pose_error(pose,this->payload.pose());
	}
	else
	{
		if(this->leftControl)
		{
			pose = this->leftTrajectory.get_pose(time);
			dx.head(3) = pose_error(pose,this->leftPose).head(3);
//			dx.head(6) = pose_error(pose,this->leftPose);
		}
//		else	Set column for torso joints to zero

		std::cout << "\nHere is the step for the left hand:" << std::endl;
		std::cout << dx.head(6) << std::endl;
		
		if(this->rightControl)
		{
			pose = this->rightTrajectory.get_pose(time);
			dx.tail(6) = pose_error(pose,this->rightPose);
			dx.tail(3).setZero();
		}
	}
	
	return dx;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Solve the step size to track the joint trajectory                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd PositionControl::track_joint_trajectory(const double &time)
{
	Eigen::VectorXd dq(this->n); dq.setZero();                                                  // Value to be returned
	
	for(int i = 0; i < this->n; i++)
	{
		dq[i] = this->jointTrajectory[i].evaluatePoint(time) - this->q[i];
	}
	
	return dq;
}

#endif
