    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                      A custom class for 2-handed control of the ergoCub                        //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


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
				 Eigen::Isometry3d(Eigen::Translation3d(0.0,0.0,0.63)*Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())))
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
ergoCub::run()
{
	update_state();                                                                             // Update kinematics & dynamics for new control loop
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since activation of control loop
	
	if(this->controlSpace == joint)
	{
		Eigen::VectorXd qd(this->n);
		Eigen::VectorXd q0(this->n);
		
		for(int i = 0; i < this->n; i++)
		{
			qd(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime);
			
			double lower = this->pLim[i][0];
			double upper = this->pLim[i][1];
			
			
/*
		Eigen::VectorXd qd(this->n);
		Eigen::VectorXd q0(this->n);
		
		for(int i = 0; i < this->n; i++)
		{
			qd(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime);                // Desired state for the given time
			
			double lower = this->pLim[i][0];
			double upper = this->pLim[i][1];

			this->z(i)         = -upper;
			this->z(i+this->n) =  lower;
			
			q0(i) = 0.5*(lower + upper);
		}
		
		this->z.tail(10) = -this->b;
		
		qRef = solve(Eigen::MatrixXd::Identity(this->n,this->n),                            // H
		            -qd,                                                                    // f
		             this->B.block(0,12,10+2*this->n,this->n),                              // Remove component for Lagrange multipliers in Cartesian mode
		             this->z,                             
		             q0);
*/	
	}
	else
	{
	
	}
}
