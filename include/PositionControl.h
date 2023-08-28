    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                         Position control functions for the iCub/ergoCub                       //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POSITION_CONTROL_H_
#define POSITION_CONTROL_H_

#include <iCubBase.h>
#include <yarp/os/BufferedPort.h>                                                                   // For publishing data
#include <yarp/sig/Vector.h>                                                                        // For publishing data

class PositionControl : public iCubBase
{
	public:
		PositionControl(const std::string              &pathToURDF,
			        const std::vector<std::string> &jointList,
			        const std::vector<std::string> &portList,
			        const std::string              &robotModel);


		//////////////////////// Inherited from iCubBase class ////////////////////////////
		bool compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum);
		
		Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time);
		
		Eigen::VectorXd track_joint_trajectory(const double &time);
		///////////////////////////////////////////////////////////////////////////////////
		
	protected:
	
		Eigen::VectorXd qRef;                                                               // Reference joint position to send to motors

		Eigen::Matrix<double,6,1> grasp_correction();                                       // Try to adjust the hand poses    
		
		yarp::os::BufferedPort<yarp::sig::Vector> actualPosition, referencePosition, positionError;                       
		
		/////////////////////// Inherited from PeriodicThread class ///////////////////////
		bool threadInit();
		void run();
		void threadRelease();
		
		//////////////////////// Methods & members specific to iCub2 //////////////////////
		Eigen::MatrixXd A;                                                                  // Constraint matrix for the iCub2
		Eigen::Matrix<double,10,1> b;                                                       // Constraint vector for the iCub2
		Eigen::MatrixXd B;
		Eigen::MatrixXd Bsmall;
		
		Eigen::VectorXd icub2_cartesian_control(const Eigen::Matrix<double,12,1> &dx,
		                                        const Eigen::VectorXd &lowerBound,
		                                        const Eigen::VectorXd &upperBound);
		                          
		Eigen::Matrix<double,12,1> lagrange_multipliers(const Eigen::Matrix<double,12,1> &dx,
                                                                const Eigen::VectorXd &redundantTask);
		
		
};                                                                                                  // Semicolon needed after class declaration

#endif
