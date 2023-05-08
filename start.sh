#!/bin/bash

SESSION="bimanual"                                                                       # So we can reference $SESSION later
SERVERNAME="/commandServer"

# Options
CONFIG="~/workspace/icub-bimanual/config/ergocub.ini"
PORT="/ergocubSim"
URDF="~/workspace/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/ergoCubGazeboV1/model.urdf"
WORLD="~/workspace/icub-bimanual/gazebo/worlds/ergocub_grasp_demo.sdf"

#CONFIG="~/workspace/icub-bimanual/config/icub2.ini"                                                             
#PORT="/icubSim"                                                                          # Port name
#URDF="~/workspace/robotology-superbuild/src/icub-models/iCub/robots/iCubGazeboV2_7/model.urdf"
#WORLD="~/workspace/icub-bimanual/gazebo/worlds/icub2_grasp_demo.sdf"                     # Location of the Gazebo world

if pgrep -x gzserver >/dev/null
	then
		echo "Gazebo server is still running!"
		killall -9 gzserver
fi

# Create first window & panel  
tmux new-session -d -s $SESSION                                                          # Start new session with given name
tmux rename-window -t 0 'Main'                                                           # Give a name to this window

# Divide up the screen (default pane 0)
#######################################
#                  #         2        #
#        0         #                  #
#                  ####################
#                  #                  #
####################         3        #
#                  #                  #
#        1         ####################
#                  #         4        #
#                  #                  #
#######################################
tmux split-window -h                                                                     # Pane 2
tmux split-window -v                                                                     # Pane 3
tmux resize-pane -U 5                                                                    # Move it up
tmux split-window -v                                                                     # Pane 4
tmux resize-pane -U 5                                                                    # Make the panel a little bigger
tmux select-pane -t 0                                                                    # Go back to 0
tmux split-window -v                                                                     # Pane 1

# Split window and run YARP
tmux select-pane -t 2
tmux send-keys -t $SESSION "yarpserver --write" Enter                                    # Start up yarp

# Split right pane, launch GAZEBO
tmux select-pane -t 3
tmux send-keys -t $SESSION "gazebo $WORLD" Enter                                         # Launch the given sdf file

# Split the pane, launch the command server
tmux select-pane -t 4
tmux send-keys -t $SESSION "~/workspace/icub-bimanual/build/bin/command_server $SERVERNAME $PORT $URDF $CONFIG" Enter

tmux attach-session -t $SESSION:0                                                        # REQUIRED or the above won't execute

