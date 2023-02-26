    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                      A custom class for 2-handed control of the ergoCub                        //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ERGOCUB_H_
#define ERGOCUB_H_

#include <PositionControl.h>

class ergoCub : public PositionControl
{
	public:
		ergoCub(const std::string &pathToURDF,
		        const std::vector<std::string> &jointNames,
		        const std::vector<std::string> &portNames);
	
	private:
	
		Eigen::VectorXd setPoint;
		
		void run();                                                                         // This is the main control loop
		
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
ergoCub::ergoCub(const std::string &pathToURDF,
		 const std::vector<std::string> &jointNames,
		 const std::vector<std::string> &portNames)
		 :
		 PositionControl(pathToURDF,
				 jointNames,
				 portNames,
				 Eigen::Isometry3d(Eigen::Translation3d(0.0,0.0,0.63)*Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())),
				 "ergocub")
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void ergoCub::run()
{
	update_state();                                                                             // Update kinematics & dynamics for new control loop
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since activation of control loop
	
	if(this->controlSpace == joint)
	{
		Eigen::VectorXd qd(this->n);
		
		for(int i = 0; i < this->n; i++)
		{
			qd(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime);
			
			if(qd(i) < this->pLim[i][0]) qd(i) = this->pLim[i][0] + 0.001;              // Just above the lower limit
			if(qd(i) > this->pLim[i][1]) qd(i) = this->pLim[i][1] - 0.001;              // Just below the upper limit
		}
		
		this->qRef = qd;                                                                    // Reference position for joint motors
	}
	else
	{
/*		Eigen::VectorXd dq(this->n);                                                        // We want to solve this
		
		// Calculate instantaneous joint limits
		Eigen::VectorXd lower(this->n), upper(this->n);
		for(int i = 0; i < this->n; i++)
		{
			compute_joint_limits(lower(i),upper(i),i);
		}
		
		Eigen::VectorXd dx = track_cartesian_trajectory(elapsedTime);                       // Get the desired Cartesian motion
		
		try // to solve the joint motion
		{
			dq = solve( 0.01*(this->setPoint - this->q),                                // Redundant task,
		                    this->M,                                                        // Weight the joint motion by the inertia,
		                    dx,                                                             // Constraint vector
		                    this->J,                                                        // Constraint matrix
		                    lower,
		                    upper,
		                    0.5*(lower + upper) );                                          // Start point
		}
		catch(const char* error_message)
		{
			std::cout << error_message << std::endl;
			dq.setZero();
		}
		
		// Resolve the last subject to grasp constraints
		if(this->isGrasping)
		{
			Eigen::MatrixXd Jc = this->C*this->J;                                       // Constraint matrix
			
			try
			{
				dq = solve( dq,
			  		    this->M,
			                    Eigen::VectorXd::Zero(6),
			                    Jc,
			                    lower,
			                    upper,
			                    0.5*(lower + upper) );
			}
			catch(const char* error_message)
			{
				std::cout << error_message << std::endl;
				dq.setZero();
			}
		}
		
		this->qRef += dq;*/
	}

	for(int i = 0; i < this->n; i++) send_joint_command(i,this->qRef[i]);
}

#endif
