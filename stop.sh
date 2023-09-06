#!/bin/bash

tmux kill-session -t 'client'
tmux kill-session -t 'server'
killall -9 gzserver
killall -9 gzclient

