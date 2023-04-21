#!/bin/bash

# Options
CONFIG="~/workspace/icub-bimanual/config/icub2.ini"                                                             
PORT="/icubSim"                                                                          # Port name
SESSION="bimanual"                                                                       # So we can reference $SESSION later
URDF="~/workspace/robotology-superbuild/src/icub-models/iCub/robots/iCubGazeboV2_7/model.urdf"
WORLD="~/workspace/icub-bimanual/gazebo/worlds/icub2_grasp_demo.sdf"                     # Location of the Gazebo world

if pgrep -x gzserver >/dev/null
	then
		echo "Gazebo server is still running!"
		killall -9 gzserver
fi

# Create first window & panel  
tmux new-session -d -s $SESSION                                                          # Start new session with given name
tmux rename-window -t 0 'Main'                                                           # Give a name to this window

# Split window

# Split window and run YARP
tmux split-window -h                                                                     # Horizontal split
tmux send-keys -t $SESSION "yarpserver --write" Enter                                    # Start up yarp

# Split right pane, launch GAZEBO
tmux split-window -v                                                                     # Vertical split
tmux resize-pane -U 5                                                                    # Move it up
tmux send-keys -t $SESSION "gazebo $WORLD" Enter                                         # Launch the given sdf file

# Split the right pane, launch the command server
tmux split-window -v                                                                     # Vertical split
tmux resize-pane -U 5                                                                    # Make the panel a little bigger
tmux send-keys -t $SESSION "~/workspace/icub-bimanual/build/bin/command_server $PORT $URDF $CONFIG" Enter


tmux attach-session -t $SESSION:0                                                        # REQUIRED or the above won't execute

