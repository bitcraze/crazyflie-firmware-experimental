#!/usr/bin/env bash

# This script uses a docker image built from https://github.com/whoenig/uav_trajectories with the name "gentrajectory"
docker run --rm -it -v $PWD/trajectory:/module gentrajectory --input /module/source_points.csv --v_max 2.0 --a_max 2.0 -o /module/trajectory_out.csv
