# Phuc Tran

## October 2021

### Introductary BFS path-finding

### Description

The program creates a path from a start to a goal using BFS (breadth-first search). Since this is not a predefined adjencent neighbors sort of map that we saw in my introductary class, we treated adjacent neighbors to that current node as the index one space away from the node in a 2-D array. The program then uses that path to convert to distances and angles for a sequence of poses. When we want to publish the markers, indicating the path the robot wants to take, we had to define a series of parameters that were needed for a Marker() message.

### Evaluation

I would say that the algorithm was effective in providing us a path from start to goal. This algorithm also has a pointer that allows us to update the path with history, improving from a DFS without history algorithm. The backpointer dictionary allows us to track what has or has not been visited. However, it is hard to evaluate with other search algorithms suchas RTT, RTT*, and A*. In the future, I definitely want to improve the search algorithm by shorterning the path with another algorithm. Once I have created a sequence of paths, I would create one segment for all steps in the same direction. Then, I can perform a line simplification algorithm like the Douglas-Peucker algorithm. However, that would also mean I would have to account for borders when I try to simplify these routes. There are many possibilities to take with this algorithm, but I chose to rely to BFS since I was already familiar with it in introductary CS courses.

### Testing

Modify line 242 and 243 in pa3.py 

1. start = (10, 10), goal = (50, 70)

2. start = (20, 20), goal = (55, 82)

3. start = (35, 15), goal = (79 12)

View Marker and Pose from rviz to verify on the path that was suggested on terminal

### Execution

1. Start docker with `docker-compose up --build`

2. Run `docker-compose exec ros bash` on four different terminals

3. Compile `roscore` on the first terminal

4. Compile `rosrun map_server map_server <correct directory>/maze.yml` on the second terminal to load a map and publish to `/map`

5. Compile `rviz` on the third terminal to start the visualization program on your local host server

6. Compile `python <correct directory>/pa3.py` on the fourth terminal

7. Add Marker and Pose topics on rviz to see the path of the robot
