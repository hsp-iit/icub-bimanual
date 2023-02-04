    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //          A trajectory across two or more poses (position & orientation) in 3D space.           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CARTESIAN_TRAJECTORY_H_
#define CARTESIAN_TRAJECTORY_H_

#include <Eigen/Geometry>                                                                           // Eigen::Isometry3d
#include <iDynTree/Core/CubicSpline.h>                                                              // Fundamental trajectory object
#include <iostream>                                                                                 // std::cout, std::cerr
#include <vector>                                                                                   // std::vector

class CartesianTrajectory
{
	public:
		CartesianTrajectory() {}                                                            // Empty constructor
		
		CartesianTrajectory(const std::vector<Eigen::Isometry3d> &poses,
		                    const std::vector<double>            &times);                   // Full constructor
		            
		bool get_state(Eigen::Isometry3d         &pose,
		               Eigen::Matrix<double,6,1> &twist,
		               Eigen::Matrix<double,6,1> &acc,
		               const double              &time);                                    // Get the desired state for the given time
	
	private:
	
		bool isValid = false;                                                               // Won't do anything if false
		
		int numPoses;                                                                       // Number of poses
		
		std::vector<iDynTree::CubicSpline> spline;                                          // Array of spline objects
		
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianTrajectory::CartesianTrajectory(const std::vector<Eigen::Isometry3d> &poses,
                                         const std::vector<double> &times) :
                                         numPoses(poses.size())
{
	// Check that the dimensions are sound before proceeding
	if(poses.size() != times.size())
	{
		std::cerr << "[ERROR] [CARTESIAN TRAJECTORY} Constructor: "
		          << "Pose vector had " << poses.size() << " elements, and "
		          << "time vector had " << times.size() << " elements!" << std::endl;
	}
	else if(poses.size() < 2)
	{
		std::cerr << "[ERROR] [CARTESIAN TRAJECTORY] Constructor: "
		          << "A minimum of 2 poses is needed to create a trajectory!" << std::endl;
	}
	else
	{
		std::vector<std::vector<double>> points; points.resize(6);                          // 6 dimension in 3D space
		for(int i = 0; i < 6; i++) points[i].resize(this->numPoses);
		
		// Extract all the positions and euler angles for each pose
		for(int i = 0; i < this->numPoses; i++)
		{
			Eigen::Vector3d translation = poses[i].translation();
			Eigen::Vector3d rotation    = poses[i].rotation().eulerAngles(0,1,2);
			
			for(int j = 0; j < 3; j++)
			{
				points[ j ][i] = translation(j);
				points[j+1][i] = rotation(j);
			}
		}
		 
		// Now insert them in to the iDynTree::CubicSpline object
		this->spline.resize(6);
		for(int i = 0; i < 6; i++)
		{
			this->spline[i].setData(iDynTree::VectorDynSize(times),
		 	                        iDynTree::VectorDynSize(points[i]));
		}
		
		this->isValid = true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Get the desired state for the given time                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CartesianTrajectory::get_state(Eigen::Isometry3d         &pose,
                                    Eigen::Matrix<double,6,1> &vel,
                                    Eigen::Matrix<double,6,1> &acc,
                                    const double              &time)
{
	if(not this->isValid)
	{
		std::cerr << "[ERROR] [CARTESIAN TRAJECTORY] get_state(): "
		          << "There was a problem during the construction of this object. "
		          << "Could not get the state." << std::endl;
		
		return false;
	}
	else
	{
		double pos[3];                                                                      // Position vector
		double rpy[3];                                                                      // Euler angles
		
		for(int i = 0; i < 3; i++)
		{
			pos[i] = this->spline[ i ].evaluatePoint(time, vel[i]  , acc[i]);
			rpy[i] = this->spline[i+3].evaluatePoint(time, vel[i+3], acc[i+3]);
		}
		
		// Now construct the pose/transform object
		pose = Eigen::Translation3d(pos[0],pos[1],pos[2])                                   // Translation first
		     * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())                          // Rotation about x
                     * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())                          // Rotation about y
                     * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());                         // Rotation about z
		
		return true;
	}
}
#endif
