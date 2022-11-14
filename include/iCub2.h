  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Custom class for 2-handed control of iCub 2                          //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iCubBase.h>                                                                               // Custom class: most functions defined here

std::vector<std::string> jointList = {"torso_pitch", "torso_roll", "torso_yaw",
			"l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw",
		        "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};
		        
class iCub2 : public iCubBase
{
	public:
		iCub2(const std::string &fileName,
		      const std::vector<std::string> &jointList,
		      const std::vector<std::string> &portList);
	
	private:

		void run();                                                                         // Main control loop
	
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Constructor for the iCub 2                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCub2::iCub2(const std::string &fileName,
             const std::vector<std::string> &jointList,
             const std::vector<std::string> &portList) :
             iCubBase(fileName, jointList, portList)
{
	std::cout << "\nWorker bees can leave.\n"
	          << "Even drones can fly away.\n"
	          << "The Queen is their slave.\n" << std::endl;
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCub2::run()
{
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since start of control loop
	
	Eigen::VectorXd vCommand(this->n); vCommand.setZero();                                      // Value to be computed
	
	if(this->controlMode == joint)
	{
		// Worker bees can leave.
	}
	else if(this->controlMode == cartesian)
	{
		// Even drones can fly away.
	}
	else if(this->controlMode == grasp)
	{
		// The Queen is their slave.
	}
	
	// Transfer the value from Eigen::Vector to std::vector<double> and send
	std::vector<double> temp; temp.resize(this->n);
	for(int i = 0; i < this->n; i++) temp[i] = vCommand(i);
	
	send_velocity_commands(temp);
}
