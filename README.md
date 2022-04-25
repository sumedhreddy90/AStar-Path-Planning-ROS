# AStar-Path-Planning-ROS Project 3- Phase 2

## Instructions to Run the program

1. Launch astarplanning package using roslaunch

```
roslaunch astarplanning astarBot.launch start_location:="[1,1,90]" goal_location:="[4,7,0]" RPM:="[3,3]"

```
2. 2D simulation 

```
- In your catkin_ws navigate to below directory

cd src/AStar-Path-Planning-ROS/src/scripts/

- Run the below python file with following args 

python simulation.py --start_location="[1,1,90]" --goal_location="[4,7,0]" --RPM="[15,15]" --isExploration=0

Note: 
- If you wanted to test the exploration of Astar Algorithm pass isExploration flag as 1 --isExploration=1

python simulation.py --start_location="[1,1,90]" --goal_location="[4,7,0]" --RPM="[15,15]" --isExploration=1


```

## Output Recordings

1. start_location:="[1,1,90]" goal_location:="[4,7,0]" RPM:="[3,3]"

#### Screen Captures:


### video recording: 

google drive link: 
