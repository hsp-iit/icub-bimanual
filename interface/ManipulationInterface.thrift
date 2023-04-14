service ManipulationInterface
{
	bool grasp_object(1:list<double> pose);                                                     # Grasp an object in a given pose
	
	bool move_object_to_pose(1:list<double> pose);                                              # Move a grasped object to a given pose
	
	bool move_hands_by_action(1:string actionName);                                             # Move hands with a pre-defined transform
	
	bool move_hands_to_pose(1:list<double> pose);                                               # Move the hands to a given pose
	
	bool move_joints_to_named_position(1:string positionName);                                  # Move the joints to (a) predefined configuration(s)
	
	bool move_joints_to_position(1:list<double> position);                                      # Move the joints to a given configuration
	
	bool release_object();                                                                      # As it says on the label
	
	bool stop();                                                                                # Stop the robot moving immediately
}
