#!/bin/bash

cd docker
docker run -itd --rm --network=host \
  --env DISPLAY=:0 --env QT_X11_NO_MITSHM=1 --env XDG_RUNTIME_DIR=/root/1000 --env XAUTHORITY=/root/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ${XAUTHORITY}:/root/.Xauthority:rw \
  -v /home/aros/projects/ergocub/ergocub-manipulation:/root/ergocub-manipulation \
  --name ergocub_manipulation_container \
  ar0s/ergocub-manipulation bash

# Create tmux session
tmux new-session -d -s manipulation-tmux

# 0
  tmux send-keys -t manipulation-tmux "docker exec -it ergocub_manipulation_container bash" Enter
  tmux send-keys -t manipulation-tmux "yarp conf 10.0.0.150 10000" Enter
#  tmux send-keys -t manipulation-tmux "yarpserver" Enter
#  tmux send-keys -t manipulation-tmux "yarp detect --write" Enter

tmux split-window -h -t manipulation-tmux


# 2
  tmux send-keys -t manipulation-tmux "docker exec -it ergocub_manipulation_container bash" Enter
  tmux send-keys -t manipulation-tmux "gazebo ~/ergocub-manipulation/gazebo/worlds/grasp-demo.sdf" Enter

tmux select-pane -t manipulation-tmux:0.0
tmux split-window -v -t manipulation-tmux

  tmux send-keys -t manipulation-tmux "docker exec -it ergocub_manipulation_container bash" Enter
  tmux send-keys -t manipulation-tmux "cd /root/ergocub-manipulation/build/bin" Enter
  tmux send-keys -t manipulation-tmux "sleep 5" Enter
  tmux send-keys -t manipulation-tmux "./grasp-demo /robotology-superbuild/build/install/share/iCub/robots/iCubGazeboV2_7/model.urdf" Enter

tmux select-pane -t manipulation-tmux:0.2
tmux split-window -v -t manipulation-tmux

  tmux send-keys -t manipulation-tmux "sleep 5" Enter
  tmux send-keys -t manipulation-tmux "docker exec -it ergocub_manipulation_container bash" Enter
  tmux send-keys -t manipulation-tmux "yarp rpc /command" Enter

tmux a -t manipulation-tmux