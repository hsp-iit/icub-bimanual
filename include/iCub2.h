

#include <iCubBase.h>

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

	virtual bool threadInit() { return true;}                                                   // Pre-processing for control loop
	virtual void run() {}                                                                       // Main control loop
	virtual void threadRelease() {}                                                             // Post-processing of control loop
	
};                                                                                                  // Semicolon needed after class declaration

iCub2::iCub2(const std::string &fileName,
             const std::vector<std::string> &jointList,
             const std::vector<std::string> &portList) :
             iCubBase(fileName, jointList, portList)
{
	std::cout << "\nWorker bees can leave.\n"
	          << "Even drones can fly away.\n"
	          << "The Queen is their slave.\n" << std::endl;
}
             
