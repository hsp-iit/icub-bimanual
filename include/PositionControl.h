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
		PositionControl(const std::string              &pathToURDF,
			        const std::vector<std::string> &jointList,
			        const std::vector<std::string> &portList,
//			        const Eigen::Isometry3d        &torsoPose,
			        const std::string              &robotModel)
		:
	        iCubBase(pathToURDF, jointList, portList, robotModel)
	        {
	        
	        	// Shoulder constraints for iCub 2
			double c = 1.71;
			this->A = Eigen::MatrixXd::Zero(10,this->numJoints);
			this->A.block(0,3,5,3) <<  c, -c,  0,
						   c, -c, -c,
						   0,  1,  1,
						  -c,  c,  c,
					 	   0, -1, -1;
							   
			this->A.block(5,10,5,3) = this->A.block(0,3,5,3);                           // Same constraint for right arm as left arm
			
			this->b.head(5) << 347.00*(M_PI/180),
					   366.57*(M_PI/180),
					    66.60*(M_PI/180),
					   112.42*(M_PI/180),
					   213.30*(M_PI/180);
					   
			this->b.tail(5) = this->b.head(5);
	        }

		// NOTE: THESE ARE INHERITED FROM ICUBBASE
		bool compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum);
		
		Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time);
		
		Eigen::VectorXd track_joint_trajectory(const double &time);
		
	protected:
		Eigen::VectorXd qRef;                                                               // Reference joint position to send to motors

		// NOTE; THESE ARE INHERITED FROM PERIODICTHREAD
		bool threadInit();
		void run();
		void threadRelease();
		
		Eigen::MatrixXd A;
		Eigen::Matrix<double,10,1> b;
};                                                                                                  // Semicolon needed after class declaration

#endif
