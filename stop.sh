#!/bin/bash

SESSION="bimanual"                                                                          # MUST match name in start.sh

tmux kill-session -t $SESSION                                                               # Kill all tmux sessions with the associated name
killall -9 gzserver                                                                         # Make sure Gazebo stops
