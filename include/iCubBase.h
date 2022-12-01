    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                       Base class for bimanual control of the iCub                              //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ICUBBASE_H_
#define ICUBBASE_H_

#include <CartesianTrajectory.h>                                                                    // Custom class
#include <Eigen/Dense>                                                                              // Tensors and matrix decomposition
#include <iDynTree/Core/EigenHelpers.h>                                                             // Converts iDynTree tensors to Eigen
#include <iDynTree/Core/CubicSpline.h>                                                              // For joint trajectories
#include <iDynTree/KinDynComputations.h>                                                            // Class for inverse dynamics calculations
#include <iDynTree/Model/Model.h>                                                                   // Class that holds basic dynamic info
#include <iDynTree/ModelIO/ModelLoader.h>                                                           // Extracts information from URDF
#include <JointInterface.h>                                                                         // Communicates with motors
#include <QPSolver.h>                                                                               // Custom class
#include <yarp/os/PeriodicThread.h>                                                                 // Keeps timing of the control loop
#include <yarp/sig/Vector.h>

class iCubBase : public yarp::os::PeriodicThread,                                                   // Regulates the control loop
                 public JointInterface,                                                             // Communicates with motor controllers
                 public QPSolver                                                                    // Used to solve joint control
{
	public:
		
		iCubBase(const std::string &fileName,
		         const std::vector<std::string> &jointList,
		         const std::vector<std::string> &portList,
		         const iDynTree::Transform      &_torsoPose);
		         
		bool move_to_pose(const yarp::sig::Matrix &left,
		                  const yarp::sig::Matrix &right,
		                  const double            &time);
		         
		bool move_to_poses(const std::vector<yarp::sig::Matrix> &left,
		                   const std::vector<yarp::sig::Matrix> &right,
		                   const std::vector<double>            &times);
		         
		bool move_to_position(const yarp::sig::Vector &position,                            // Move joints to given position in a given time
		                      const double            &time);

		bool move_to_positions(const std::vector<yarp::sig::Vector> &positions,             // Move joints through multiple positions
				       const std::vector<double>            &times);
				       
		bool print_hand_pose(const std::string &which);                                     // As it says on the label
		
		bool set_cartesian_gains(const double &stiffness, const double &damping);
				       
		bool set_joint_gains(const double &proportional, const double &derivative);         // As it says on the label
		
		bool translate(const yarp::sig::Vector &left,                                       // Translate both hands by the given amount
		               const yarp::sig::Vector &right,
		               const double            &time);
				      
		void halt();
	
	protected:
	
		double dt = 0.01;                                                                   // Default control frequency
		double maxAcc = 10;                                                                 // Limits the acceleration
		double startTime;                                                                   // Used for timing the control loop
		
		Eigen::VectorXd q, qdot;                                                            // Joint positions and velocities
		
		enum ControlSpace {joint, cartesian} controlSpace;
		enum ControlMode  {velocity, torque} controlMode;
		
		// Joint control properties
		double kq = 10.0;                                                                   // Feedback on joint position error
		double kd =  5.0;                                                                   // Feedback on joint velocity error
		std::vector<iDynTree::CubicSpline> jointTrajectory;                                 // Generates reference trajectories for joints
		
		// Cartesian control
		bool leftControl, rightControl;                                                     // Switch for activating left and right control
		CartesianTrajectory leftTrajectory, rightTrajectory;                                // Individual trajectories for left, right hand
		Eigen::Matrix<double,6,6> K;                                                        // Feedback on pose error
		Eigen::Matrix<double,6,6> D;                                                        // Feedback on velocity error
		Eigen::Matrix<double,6,6> gainTemplate;                                             // Structure for the Cartesian gains
		Eigen::MatrixXd J, M;
		
		// Kinematics & dynamics
		iDynTree::KinDynComputations computer;                                              // Does all the kinematics & dynamics
		iDynTree::Transform          torsoPose;                                             // Needed for inverse dynamics; not used yet
		iDynTree::Twist              torsoTwist;                                            // Needed for inverse dynamics; not used yet
		iDynTree::Vector3            gravity;                                               // Needed for inverse dynamics; not used yet
		
		// Variables for the QP solver
		Eigen::MatrixXd H, B;
		
			                       	
		// Internal functions
		bool get_speed_limits(double &lower, double &upper, const int &i);                  // Get velocity/acceleration limits
		
		bool update_state();                                                                // Get new joint state, update kinematics
		                                 
		Eigen::VectorXd track_joint_trajectory();                                           // Solve feedforward + feedback control         
		
		Eigen::VectorXd track_cartesian_trajectory();                                       // Solve feedforward + feedback control
		
		yarp::sig::Vector get_pose_error(const yarp::sig::Matrix &desired,
		                                 const yarp::sig::Matrix &actual);                  // As it says on the label
		
		yarp::sig::Matrix convert_iDynTree_to_yarp(const iDynTree::Transform &T);
		
		// Functions related to the PeriodicThread class
		bool threadInit();
		void threadRelease();
//              void run(); // This is declared in the child class since it can be unique
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         CONSTRUCTOR                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCubBase::iCubBase(const std::string &fileName,
                   const std::vector<std::string> &jointList,
                   const std::vector<std::string> &portList,
                   const iDynTree::Transform &_torsoPose) :
                   yarp::os::PeriodicThread(0.01),                                                  // Create thread to run at 100Hz
                   JointInterface(jointList, portList),                                             // Communicates with joint motors
                   torsoPose(_torsoPose),
                   torsoTwist(iDynTree::Twist(iDynTree::GeomVector3(0,0,0),iDynTree::GeomVector3(0,0,0)))
{
	// Set the gravity vector
	this->gravity(0) =  0.00;
	this->gravity(1) =  0.00;
	this->gravity(2) = -9.81;

	// Set the Cartesian control gains	
	this->gainMatrix << 1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		            0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
		            0.0,   0.0,   1.0,   0.0,   0.0,   0.0,
		            0.0,   0.0,   0.0,   0.1,   0.0,   0.0,
		            0.0,   0.0,   0.0,   0.0,   0.1,   0.0,
		            0.0,   0.0,   0.0,   0.0,   0.0,   0.1;
	
	this->K = 10*this->gainMatrix;                                                              // Set the spring forces
	this->D =  5*this->gainMatrix;                                                              // Set the damping forces
	
	this->J.resize(12,this->n);
	this->M.resize(this->n,this->n);
	
	// Set the constraint matrices for the QP solver
	this->startPoint.resize(this->n);                                                           // As it says on the label
	
	// H = [ 0  J ]
	//     [ J' M ]
	this->H = Eigen::MatrixXd::Zero(12+this->n,12+this->n);                                     // Upper corner never changes
	
	// B = [ 0  -I ]
	//     [ 0   I ]
	this->B.resize(2*this->n,12+this->n);
	
	this->B.block(      0, 0,this->n,     12) = Eigen::MatrixXd::Zero(this->n,12);
	this->B.block(      0,12,this->n,this->n) =-Eigen::MatrixXd::Identity(this->n,this->n);
	this->B.block(this->n, 0,this->n,     12) = Eigen::MatrixXd::Zero(this->n,12);
	this->B.block(this->n,12,this->n,this->n) = Eigen::MatrixXd::Identity(this->n,this->n);
	
	// z = [ -upperBound ]
	//     [  lowerBound ]
	this->z.resize(2*this->n); // NOTE: this vector is dynamic, so there's no point setting it here

	// Load a model
	iDynTree::ModelLoader loader;                                                               // Temporary object

	if(not loader.loadReducedModelFromFile(fileName, jointList, "urdf"))
	{
		std::cerr << "[ERROR] [ICUB] Constructor: "
		          << "Could not load model from the given path " << fileName << std::endl;
	}
	else
	{
		// Get the model and add some additional frames for the hands
		iDynTree::Model temp = loader.model();
		
		temp.addAdditionalFrameToLink("l_hand", "left",
		                              iDynTree::Transform(iDynTree::Rotation::RPY(0,0,0),
		                                                  iDynTree::Position(0.06,0,0)));
		                                                  
		temp.addAdditionalFrameToLink("r_hand", "right",
		                              iDynTree::Transform(iDynTree::Rotation::RPY(0,0,M_PI),
		                                                  iDynTree::Position(-0.06,0,0)));
		
		// NOTE: These worked for iCub3. Need to add an option to change these based on
		// the robot model.
//		temp.addAdditionalFrameToLink("l_hand", "left",
//					      iDynTree::Transform(iDynTree::Rotation::RPY(0,M_PI/2,0.0),
//								  iDynTree::Position(0,0,-0.06	)));
//		temp.addAdditionalFrameToLink("r_hand", "right",
//					      iDynTree::Transform(iDynTree::Rotation::RPY(0,M_PI/2,0.0),
//					      			  iDynTree::Position(0,0,-0.06)));	    
		if(not this->computer.loadRobotModel(temp))
		{
			std::cerr << "[ERROR] [ICUB] Constructor: "
				  << "Could not generate iDynTree::KinDynComputations class from given model: "
				  << loader.model().toString() << std::endl;
		}
		else
		{
			this->n = this->computer.model().getNrOfDOFs();                             // Get number of joints from underlying model

			// Resize vectors based on model information
			this->jointTrajectory.resize(this->n);                                      // Trajectory for joint motion control
			this->q.resize(this->n);                                                    // Vector of measured joint positions
			this->qdot.resize(this->n);                                                 // Vector of measured joint velocities

			std::cout << "[INFO] [ICUB] Successfully created iDynTree model from "
			          << fileName << "." << std::endl;

			update_state();                                                             // Get the current joint state
			
			if(not activate_control())
			{
				std::cerr << "[ERROR] [ICUB] Constructor: "
                                          << "Could not activate joint control." << std::endl;
                        }
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Move each hand to a desired pose                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_pose(const yarp::sig::Matrix &left,
                            const yarp::sig::Matrix &right,
                            const double            &time)
{
	if(left.rows() != 4 or left.cols() != 4 or right.rows() != 4 or right.cols() != 4)
	{
		std::cerr << "[ERROR] [iCUB] move_to_pose(): "
		          << "Expected 4x4 matrices for the input arguments, "
		          << "but the left argument was " << left.rows() << "x" << left.cols() << " "
		          << "and the right argument was " << right.rows() << "x" << right.cols() << "." << std::endl;
		
		return false;
	}
	else
	{
		// Put them in std::vector container and pass onward
		std::vector<yarp::sig::Matrix> leftPoses; leftPoses.push_back(left);
		std::vector<yarp::sig::Matrix> rightPoses; rightPoses.push_back(right);
		std::vector<double> times; times.push_back(time);
		
		return move_to_poses(leftPoses,rightPoses,times);
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Move both hands through multiple poses                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_poses(const std::vector<yarp::sig::Matrix> &left,
                             const std::vector<yarp::sig::Matrix> &right,
                             const std::vector<double> &times)
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	this->controlSpace = cartesian;                                                             // Switch to Cartesian control mode
	this->leftControl = true; this->rightControl = true;                                        // Activate both hands
	
	// Set up the times for the trajectory
	std::vector<double> t; t.push_back(0.0);                                                    // Start immediately
	t.insert(t.end(),times.begin(),times.end());                                                // Add on the rest of the times
	
	// Set up the left hand trajectory
	yarp::sig::Matrix T = convert_iDynTree_to_yarp(this->computer.getWorldTransform("left"));
	std::vector<yarp::sig::Matrix> waypoint; waypoint.push_back(T);                             // First waypoint is current pose
	waypoint.insert(waypoint.end(),left.begin(),left.end());                                    // Add additional waypoints to the end
	this->leftTrajectory = CartesianTrajectory(waypoint, t);                                    // Create the left-hand trajectory
	
	// Set up the right hand trajectory
	T = convert_iDynTree_to_yarp(this->computer.getWorldTransform("right"));
	waypoint.clear();
	waypoint.push_back(T);                                                                      // First waypoint is the current pose
	waypoint.insert(waypoint.end(),right.begin(),right.end());                                  // Add additional poses to the end
	this->rightTrajectory = CartesianTrajectory(waypoint,t);                                    // Create new trajectory for the right hand
	
	start(); // Go immediately to threadInit();

	return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Move the joints to a desired configuration                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_position(const yarp::sig::Vector &position,
                                const double &time)
{
	if(position.size() != this->n)
	{
		std::cerr << "[ERROR] [ICUB] move_to_position(): "
			  << "Position vector had " << position.size() << " elements, "
			  << "but this model has " << this->n << " joints." << std::endl;
			  
		return false;
	}
	else
	{
		std::vector<yarp::sig::Vector> target; target.push_back(position);                  // Insert in to std::vector to pass onward
		std::vector<double> times; times.push_back(time);                                   // Time in which to reach the target position
		return move_to_positions(target,times);                                             // Call "main" function
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Move the joints to several desired configurations at given time                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_positions(const std::vector<yarp::sig::Vector> &positions,
                                 const std::vector<double> &times)
{
	if(positions.size() != times.size())
	{
		std::cout << "[ERROR] [ICUB] move_to_positions(): "
		          << "Position array had " << positions.size() << " waypoints, "
		          << "but the time array had " << times.size() << " elements!" << std::endl;

		return false;
	}
	else
	{
		if(isRunning()) stop();                                                             // Stop any control thread that might be running
		this->controlSpace = joint;                                                         // Switch to joint control mode
		int m = positions.size() + 1;                                                       // We need to add 1 extra waypoint for the start
		iDynTree::VectorDynSize waypoint(m);                                                // All the waypoints for a single joint
		iDynTree::VectorDynSize t(m);                                                       // Times to reach the waypoints
		
		for(int i = 0; i < this->n; i++)                                                    // For the ith joint...
		{
			for(int j = 0; j < m; j++)                                                  // ... and jth waypoint
			{
				if(j == 0)
				{
					waypoint[j] = this->q[i];                                   // Current position is start point
					t[j] = 0.0;                                                 // Start immediately
				}
				else
				{
					double target = positions[j-1][i];                                  // Get the jth target for the ith joint
					
					     if(target < this->pLim[i][0]) target = this->pLim[i][0] + 0.01;// Just above the lower limit
					else if(target > this->pLim[i][1]) target = this->pLim[i][1] - 0.01;// Just below the upper limit
					
					waypoint[j] = target;                                       // Assign the target for the jth waypoint
					
					t[j] = times[j-1];                                          // Add on subsequent time data
				}
			}
			
			if(not this->jointTrajectory[i].setData(t,waypoint))
			{
				std::cerr << "[ERROR] [ICUB] move_to_positions(): "
				          << "There was a problem setting new joint trajectory data." << std::endl;
			
				return false;
			}
		}
		
		start();                                                                            // Start the control thread
		return true;                                                                        // Success
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Print the pose of a hand to the console                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::print_hand_pose(const std::string &which)
{
	if(which == "left" or which == "right")
	{
		std::cout << "Here is the " << which << " hand pose:" << std::endl;
		std::cout << this->computer.getWorldTransform(which).asHomogeneousTransform().toString() << std::endl;

		return true;
	}
	else
	{
		std::cout << "[ERROR] [iCUB] print_hand_pose(): " 
		          << "Expected 'left' or 'right' as the argument, "
		          << "but the input was " << which << "." << std::endl;
		
		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Set the gains for control in Cartesian space                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_cartesian_gains(const double &stiffness, const double &damping)
{
	if(stiffness <= 0 or damping <= 0)
	{
		std::cerr << "[ERROR] [iCUB] set_cartesian_gains(): "
		          << "Gains cannot be negative! "
		          << "You input " << stiffness << " for the stiffness gain, "
		          << "and " << damping << " for the damping gain." << std::endl;
		
		return false;
	}
	else
	{	
		this->K = stiffness*this->gainMatrix;
		this->D =   damping*this->gainMatrix;
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Set the gains for control in the joint space                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_joint_gains(const double &proportional, const double &derivative)
{
	if(proportional <= 0 or derivative <= 0)
	{
		std::cerr << "[ERROR] [ICUB] set_joint_gains(): "
                          << "Gains cannot be negative! "
                          << "You input " << proportional << " for the proportional gain, "
                          << "and " << derivative << " for the derivative gain." << std::endl;
                
                return false;
        }
        else
        {
        	this->kq = proportional;
        	this->kd = derivative;
        	return true;
        }
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Translate both hands by the given amount                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::translate(const yarp::sig::Vector &left,
                         const yarp::sig::Vector &right,
                         const double            &time)
{
	yarp::sig::Matrix leftTarget  = convert_iDynTree_to_yarp(this->computer.getWorldTransform("left"));
	yarp::sig::Matrix rightTarget = convert_iDynTree_to_yarp(this->computer.getWorldTransform("right"));
	
	for(int i = 0; i < 3; i++)
	{
		 leftTarget[i][3] +=  left[i];
		rightTarget[i][3] += right[i];
	}

	return move_to_pose(leftTarget, rightTarget,time);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Stop the robot immediately                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCubBase::halt()
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	
	for(int i = 0; i < this->n; i++) send_velocity_command(0.0, i);                             // Stop the joints from moving
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::threadInit()
{
	this->startTime = yarp::os::Time::now();
	return true;
	// jump immediately to run();
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCubBase::threadRelease()
{
	if(this->controlMode == velocity)
	{
		std::vector<double> command;
		for(int i = 0; i < this->n; i++) command.push_back(0.0);                            // Set all as zero
		send_velocity_commands(command);                                                    // Pass on to JointInterface
	}
	else if(this->controlMode == torque)
	{
		std::cerr << "This hasn't been programmed yet you fool!" << std::endl;
	}
	else
	{
		std::cerr << "[ERROR] [iCUB] threadRelease(): "
		          << "Control mode incorrectly specified! How did that happen?" << std::endl;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the error between a desired and actual pose                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
yarp::sig::Vector iCubBase::get_pose_error(const yarp::sig::Matrix &desired, const yarp::sig::Matrix &actual)
{
	yarp::sig::Vector error(6);
	
	if(desired.rows() != 4 or desired.cols() != 4 or actual.rows() != 4 or actual.cols() != 4)
	{
		std::cerr << "[ERROR] [iCUB] get_pose_error(): "
		          << "Expected 4x4 matrices for the input arguments, "
		          << "but the desired input was " << desired.rows() << "x" << desired.cols() << " "
		          << "and the actual was " << actual.rows() << "x" << actual.cols() << "." << std::endl;
		
		error.zero();
	}
	else
	{
		for(int i = 0; i < 3; i++) error(i) = desired(i,3) - actual(i,3);                   // Position/translation error
		
		yarp::sig::Matrix Re(3,3); Re.zero(); // = desired*actual.transposed()
		for(int i = 0; i < 3; i++)
		{
			for(int j = 0; j < 3; j++)
			{
				for(int k = 0; k < 3; k++) Re[i][k] += desired[i][j]*actual[k][j];
			}
		}
		
		// "Unskew" the rotation error
		error(3) = Re(2,1);
		error(4) = Re(0,2);
		error(5) = Re(1,0);
	}
	
	return error;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Get the instantenous speed limits for joint limit avoidance                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::get_speed_limits(double &lower, double &upper, const int &i)
{
	if(this->controlMode == velocity)
	{
		// Compute lower limit
		minSpeed = std::max( (this->pLim[i][0] - this->q[i])/this->dt,
			   std::max( -this->vLim[i],
			             -sqrt(2*maxAcc*(this->q[i] - this->pLim[i][0]))));
			      
		// Compute upper limit
		maxSpeed = std::min( (this->pLim[i][1] - this->q[i])/this->dt,
			   std::min(  this->vLim[i],
			              sqrt(2*maxAcc*(this->pLim[i][1] - this->q[i]))));
			              
		return true;
	}
	else if(this->controlMode == torque)
	{
		std::cerr << "This hasn't been programmed yet you fool!" << std::endl;
		
		return false;
	}
	else
	{
		std::cerr << "[ERROR] [iCUB] get_speed_limits(): "
		          << "Control mode incorrectly specified! How did that happen?" << std::endl;
	
		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Update the kinematics & dynamics of the robot                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::update_state()
{
	if(JointInterface::read_encoders())
	{	
		// Get the values from the JointInterface class (is there a smarter way???)
		std::vector<double> temp_position = get_joint_positions();
		std::vector<double> temp_velocity = get_joint_velocities();
		
		for(int i = 0; i < this->n; i++)
		{
			// Transfer joint state values for use in this class
			this->q[i]    = temp_position[i];
			this->qdot[i] = temp_velocity[i];
			
			// Update joint constraint vector for use by QP solver
			double lower, upper;
			
			if(this->controlMode == velocity) get_speed_limits(lower, upper);
			else                              get_acceleration_limits(lower, upper);
	
			this->z(i)         =-upper;
			this->z(i+this->n) = lower;
			
			this->startPoint(i) = 0.5*(upper + lower);                                  // Set as midpoint
		}
		
		// Put them in to the iDynTree class to solve all the physics
		if(this->computer.setRobotState(this->torsoPose,
		                                iDynTree::VectorDynSize(temp_position),
		                                this->torsoTwist,
		                                iDynTree::VectorDynSize(temp_velocity),
		                                this->gravity))
		{
			Eigen::MatrixXd temp;                                                       // Temporary storage
			
			// Get the left hand component
			this->computer.getFrameFreeFloatingJacobian("left",temp);                   // Compute left hand Jacobian
			this->J.block(0,0,6,this->n) = temp.block(0,6,6,this->n);                   // Assign to larger matrix
			if(not this->leftControl) this->J.block(0,0,6,3).setZero();                 // Remove torso joints
			
			// Get the right hand component
			this->computer.getFrameFreeFloatingJacobian("right",temp);                  // Compute right hand Jacobian
			this->J.block(6,0,6,this->n) = temp.block(0,6,6,this->n);                   // Assign to larger matrix
			if(not this->rightControl) this->J.block(6,0,6,3).setZero();                // Remove torso joints
			
			// Compute inertia matrix
			this->computer.getFreeFloatingMassMatrix(temp);                             // Compute full inertia matrix
			this->M = temp.block(6,6,this->n,this->n);                                  // Remove floating base
			
			// Update Hessian matrix for QP solver
//			this->H.block( 0, 0,     12,     12) = Eigen::MatrixXd::Zero(12,12);
			this->H.block( 0,12,     12,this->n) = this->J;
			this->H.block(12, 0,this->n,     12) = this->J.transposed();
			this->H.block(12,12,this->n,this->n) = this->M;
			
			return true;
		}
		else
		{
			std::cerr << "[ERROR] [ICUB] update_state(): "
				  << "Could not set state for the iDynTree::iKinDynComputations object." << std::endl;
				  
			return false;
		}
	}
	else
	{
		std::cerr << "[ERROR] [ICUB] update_state(): "
			  << "Could not update state from the JointInterface class." << std::endl;
			  
		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Solve the Cartesian control                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd<double,12,1> iCubBase::track_cartesian_trajectory(const double &time)
{
	// Variables used in this scope
	Eigen::Vector<double,12,1> ref; ref.setZero();                                              // Value to be returned
	yarp::sig::Matrix pose(4,4);                                                                // Desired pose for a hand
	yarp::sig::Vector poseError(6);                                                             // As it says on the label;
	yarp::sig::Vector vel(6), acc(6);                                                           // Desired velocity, acceleration
	iDynTree::Transform T;                                                                      // Temporary storage

	// NOTE TO SELF: There's probably a smarter way to do all this...
	
	if(this->leftControl)
	{
		this->leftTrajectory.get_state(pose,vel,acc,time);                                  // Get the desired state for the left hand
		T = this->computer.getWorldTransform("left");                                       // Get current pose of left hand
		poseError = get_pose_error(pose,convert_iDynTree_to_yarp(T));		            // Compute pose error
		
		if(this->controlMode == velocity)
		{
			for(int i = 0; i < 6; i++) ref[i] = vel[i]                                  // Velocity feedforward
			                                  + this->K(i,i)*poseError[i];              // Pose feedback
		}
		else
		{
			for(int i = 0; i < 6; i++) ref[i] = acc[i]                                  // Acceleration feedforward
			                                  + this->D(i,i)*
			                                  + this->
	
	if(this->controlMode == velocity)
	{
		if(this->leftControl)
		{
			this->leftTrajectory.get_state(pose,vel,acc,time);                          // Get the desired state for the left hand
			T = this->computer.getWorldTransform("left");                               // Get current pose of left hand
			poseError = get_pose_error(pose,convert_iDynTree_to_yarp(T));		    // Compute pose error
			for(int i = 0; i < 6; i++) ref[i] = vel[i] + this->K(i,i)*poseError[i];     // Compute feedforward + feedback control
		}
		if(this->rightControl)
		{
			this->rightTrajectory.get_state(pose,vel,acc,time);                         // Get the desired state for the right hand
			T = this->computer.getWorldTransform("right");                              // Get current pose of the right hand
			poseError = get_pose_error(pose,convert_iDynTree_to_yarp(T));               // Compute pose error
			for(int i = 0; i < 6; i++) ref[i+6] = ve[i] + this->K(i,i)*poseError[i];    // Feedforward + feedback control
		}
	}
	else if(this->controlMode == torque)
	{
		if(this->leftControl)
		{
			this->leftTrajectory.get_state(pose,vel,acc,time);
			T = this->computer.getWorldTransform("left");
			poseError = get_pose_error(pose,convertiDynTree_to_yarp
	{
	
	return ref;
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Solve the joint control                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd iCubBase::track_joint_trajectory(const double &time)
{
	Eigen::VectorXd ref(this->n);                                                               // Value to be computed
	double q_d, qdot_d, qddot_d;                                                                // Desired state
	
	if(this->controlMode == velocity)
	{
		for(int i = 0; i < this->n; i++)
		{
			q_d = this->jointTrajectory[i].evaluatePoint(time, qdot_d, qddot_d);        // Get the desired state
			
			ref[i] = qdot_d                                                             // Velocity feedforward term
			       + this->kp*(q_d - this->q[i]);                                       // Position feedback
		}
	}
	else if(this->controlMode == torque)
	{
		for(int i = 0; i < this->n; i++)
		{
			q_d = this->jointTrajectory[i].evaluatePoint(time, qdot_d, qddot_d);        // Get the desired state
			
			ref[i] = qddot_d                                                            // Acceleration feedforward
			       + this->kd*(qdot_d - this->qdot[i])                                  // Velocity feedback
			       + this->kp*(q_d - this->q[i]);                                       // Position feedback
		}
	}
	
	return ref;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Convert iDynTree::Transform to yarp::sig::Matrix                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
yarp::sig::Matrix iCubBase::convert_iDynTree_to_yarp(const iDynTree::Transform &T)
{
	iDynTree::Matrix4x4 A = T.asHomogeneousTransform();
	yarp::sig::Matrix temp(4,4); temp.eye();
	
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 4; j++) temp(i,j) = A(i,j);
	}
	
	return temp;
}

#endif
