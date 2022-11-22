    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //          A trajectory across two or more poses (position & orientation) in 3D space.           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <iDynTree/Core/CubicSpline.h>                                                              // Fundamental trajectory class
#include <iostream>                                                                                 // std::cerr and std::cout
#include <vector>                                                                                   // std::vector
#include <math.h>                                                                                   // atan2, cos, sin
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

class CartesianTrajectory
{
	public:
		CartesianTrajectory () {}                                                           // Empty constructor
		
		CartesianTrajectory(const std::vector<yarp::sig::Matrix> &poses,                    // Constructor from inputs
		                    const std::vector<double> &times);

		bool get_state(yarp::sig::Matrix &pose,                                             // Get the desired state for the given time
		               yarp::sig::Vector &twist,
		               yarp::sig::Vector &acc,
		               const double &time);
		               
	private:
		bool isValid = false;                                                               // Won't do computations if this is false
		int n;                                                                              // Number of waypoints
		std::vector<iDynTree::CubicSpline> spline;                                          // Array of splines
	
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianTrajectory::CartesianTrajectory(const std::vector<yarp::sig::Matrix> &poses,
                                         const std::vector<double> &times) :
                                         n(poses.size())
{
	// Check the dimensions are sound before proceeeding
	if(poses.size() != times.size())
	{
		std::cerr << "[ERROR] [CARTESIAN TRAJECTORY] Constructor: "
		          << "Arguments are not of equal length! "
		          << "There were " << poses.size() << " poses "
		          << "and " << times.size() << " times." << std::endl;
	}
	else if(poses.size() < 2)
	{
		std::cerr << "[ERROR] [CARTESIAN TRAJECTORY] Constructor: "
		          << "A minimum of 2 poses is needed to create a trajectory!" << std::endl;
	}
	else
	{
		std::vector<std::vector<double>> points; points.resize(6);                          // 6 dimensions
		for(int i = 0; i < 6; i++) points[i].resize(this->n);                               // n waypoints for each dimensions
		
		// Extract all the positions and euler angles for each pose
		for(int i = 0; i < this->n; i++)
		{
			if(poses[i].rows() != 4 and poses[i].cols() != 4)
			{
				std::cerr << "[ERROR] [CARTESIAN TRAJECTORY] Constructor: "
					  << "Expected a 4x4 matrix for pose " << i+1 << ", "
					  << "but it had " << poses[i].rows() << "rows "
					  << "and " << poses[i].cols() << " columns." << std::endl;
				break;
			}
			
			points[0][i] = poses[i](0,3);                                               // Get the x position
			points[1][i] = poses[i](1,3);                                               // Get the y position
			points[2][i] = poses[i](2,3);                                               // Get the z position
			
		 	yarp::sig::Matrix R = poses[i].submatrix(0,2,0,2);                          // Get the rotation matrix
		 	
		 	
		 	double roll, pitch, yaw;
		 	if(abs(R[0][2]) != 1)
		 	{
				pitch = asin(R[0][2]);
				roll  = atan2(-R[1][2],R[2][2]);
				yaw   = atan2(-R[0][1],R[0][0]);
			}
			else // Gimbal lock; yaw - roll = atan2(R[1][0],R[1][1])
			{
				pitch =-M_PI/2;
				roll  =-atan2(R[1][0],R[1][1]);
				yaw   = 0;
			}
		 	
		 	points[3][i] = roll;
		 	points[4][i] = pitch;
		 	points[5][i] = yaw;
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
bool CartesianTrajectory::get_state(yarp::sig::Matrix &pose,
                                    yarp::sig::Vector &vel,
                                    yarp::sig::Vector &acc,
                                    const double      &time)
{
	if(not this->isValid)
	{
		std::cerr << "[ERROR] [CARTESIAN TRAJECTORY] get_state(): "
		          << "There was a problem during the construction of this object. "
		          << "Could not get the state." << std::endl;
		
		return false;
	}
	else if(vel.size() != 6 or acc.size() != 6)
	{
		std::cerr << "[ERROR] [CARTESIAN TRAJECTORY] get_state(): "
		          << "Expected 6x1 vectors for the input arguments, "
		          << "but the velocity argument had " << vel.size() << " elements, "
		          << "and the acceleration argument had " << acc.size() << " elements.";
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
		
		// Now construct the homogeneous transformation matrix
		pose.resize(4,4); pose.eye();                                                       // Set to 4x4 identity to start
		for(int i = 0; i < 3; i++) pose[i][3] = pos[i];                                     // Assign the position
		
		// Convert RPY euler angles to rotation matrix
		pose[0][0] = cos(rpy[1])*cos(rpy[2]);
		pose[1][0] = sin(rpy[0])*sin(rpy[1])*cos(rpy[2]) + cos(rpy[0])*sin(rpy[2]);
		pose[2][0] =-cos(rpy[0])*sin(rpy[1])*cos(rpy[2]) + sin(rpy[0])*sin(rpy[2]);
		
		pose[0][1] =-cos(rpy[1])*sin(rpy[2]);
		pose[1][1] =-sin(rpy[0])*sin(rpy[1])*sin(rpy[2]) + cos(rpy[0])*cos(rpy[2]);
		pose[2][1] = cos(rpy[0])*sin(rpy[1])*sin(rpy[2]) + sin(rpy[0])*cos(rpy[2]);
		
		pose[0][2] = sin(rpy[1]);
		pose[1][2] =-sin(rpy[0])*cos(rpy[1]);
		pose[2][2] = cos(rpy[0])*cos(rpy[1]);
		
		return true;
	}
}
#endif
