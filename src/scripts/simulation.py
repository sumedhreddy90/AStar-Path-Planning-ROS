#!/usr/bin/env python
import math
import numpy as np
from heapq import heappush, heappop
import time
import matplotlib.pyplot as plt
import argparse
import pathplanner


if __name__ == '__main__':
	# Getting data from the user
	input_parser = argparse.ArgumentParser()
	input_parser.add_argument('--start_location', default="[1, 1, 90]")
	input_parser.add_argument('--goal_location', default="[4, 7, 0]")
	input_parser.add_argument('--robot_radius', default=0.177)
	input_parser.add_argument('--clearance', default=0.1)
	input_parser.add_argument('--theta_step', default=30)
	input_parser.add_argument('--step_size', default=2)
	input_parser.add_argument('--goal_thres', default=0.1)
	input_parser.add_argument('--wheel_radius', default=0.028)
	input_parser.add_argument('--wheel_length', default=0.320)
	input_parser.add_argument('--RPM', default="[3,3]")
	input_parser.add_argument('--weight', default=1)
	input_parser.add_argument('--isExploration', default=0)
	input_parser.add_argument('--isPathDisplay', default=1)
	input_parser.add_argument('--thresh', default=0.01)
	args = input_parser.parse_args()

	# assigning variables to input arg data
	start_location = args.start_location
	goal_location = args.goal_location
	step_size = int(args.step_size)
	thresh = float(args.thresh)
	goal_thres = float(args.goal_thres)

	# defining robot radius and clearance
	r = float(args.robot_radius)
	c = float(args.clearance)

	# organising source and goal location
	#[x, y, theta]
	source_location = [float(i) for i in start_location[1:-1].split(',')]
	#[x, y]
	goal_location = [float(i) for i in goal_location[1:-1].split(',')] 

	print("Source location:", source_location)
	print("Goal location:", goal_location)

	#non-holonomic constraints
	# Defining Differential Drive Constraints 
	rpm = [float(i) for i in args.RPM[1:-1].split(',')]
	Ur, Ul = rpm[0], rpm[1]
	wheel_length = float(args.wheel_length) 
	wheel_radius = float(args.wheel_radius)
	weight = float(args.weight)

	# Planning the goal position using non-holonomic A star algorithm
	path_planner = pathplanner.AStarAlgorithm(source_location, goal_location, step_size=step_size,
		goal_thres = goal_thres, width = 10, height = 10, thresh = thresh,
		r=r, c=c, wheel_length = wheel_length, Ur = Ur, Ul = Ul, wheel_radius = wheel_radius,
		weight = weight, isExploration=int(args.isExploration), isPathDisplay=int(args.isPathDisplay))

	#Finding the optimal path
	path_planner.computeOptimalPath()

	#Back tracing the optimal path
	print(path_planner.trace_index)

	l = wheel_length
	r = wheel_radius

	# Printing Diffrential drive parameters
	for idx in path_planner.trace_index:
		ul, ur = path_planner.actions[int(idx)] 
		dx = r*0.5*(ur+ul)
		dt = r*(ur-ul)/l
		print(r, l, ul,ur,dx,dt)



