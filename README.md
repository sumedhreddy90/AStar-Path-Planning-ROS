# AStar-Path-Planning-ROS Project 3- Phase 2

## Instructions to Run the program

1. Launch astarplanning package using roslaunch

```
roslaunch astarplanning astarBot.launch start_location:="[1,1,90]" goal_location:="[4,7,0]" RPM:="[3,3]"

```
2. 2D simulation 

- In your catkin_ws navigate to below directory
```
cd src/AStar-Path-Planning-ROS/src/scripts/

```
- Run the below python file with following args 

```
python simulation.py --start_location="[1,1,90]" --goal_location="[4,7,0]" --RPM="[15,15]" --isExploration=0

```

Note: 
- If you wanted to test the exploration of Astar Algorithm pass isExploration flag as 1 --isExploration=1
```
python simulation.py --start_location="[1,1,90]" --goal_location="[4,7,0]" --RPM="[15,15]" --isExploration=1
```


## Output Recordings

1. start_location:="[1,1,90]" goal_location:="[4,7,0]" RPM:="[3,3]"

#### Screen Captures:


![turtlebot3](https://user-images.githubusercontent.com/24978535/165004260-1b497d2b-c86c-4908-9a2a-8fa8c88026c5.png)

![exploration_map](https://user-images.githubusercontent.com/24978535/165004265-c3197124-93d3-41d1-8cd1-5cd27197ab40.png)

![path_plan](https://user-images.githubusercontent.com/24978535/165004358-06c1c20f-e99f-495b-babd-3ecdfce761ff.png)



### video recording: 

google drive link: https://drive.google.com/drive/folders/1V2lNuAGtsI3IYEYmuJ0QiLSDFob7bPlP?usp=sharing

## Contributors

1. Sumedh Reddy Koppula - 117386066
2. Ashuthosh Reddy - 118442129
