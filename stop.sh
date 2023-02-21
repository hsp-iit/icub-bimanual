#!/bin/bash

docker rm -f ergocub_manipulation_container
tmux kill-session -t manipulation-tmux
