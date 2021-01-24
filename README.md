
# __A* algorithm implementation__

A* algorithm is one of the most efficient graph traversal and path search algorithm.It starts the search from starting node and aims to find the path with lowest total cost. It maintains a tree(min-heap) of nodes to be examined with costs increasing down the tree.At each node,it extends one edge at a time upto all possible unexamined nodes and then add those to our tree along with their total cost. The next node to be examined is always the root node of the tree as it has minimum cost.The cost function of A* includes the cost incurred in visiting the node as well as a heuristic cost which estimates the value of the cheapest path from current node to goal node.The Heuristic search bias makes the algorithm more efficent as it tends to decrease the number of nodes to be processed in order to reach the goal node.

In this project,the algorithm was employed to find optimal path between two user given points from a static grid map of the environment.
Also, a proportional controller was used for motion of the bot on the obtained path.

- Simulation was done on Gazebo simulator (version 7.0.0).
- Occupancy grid of the environment was created using costmap_2d package.Obstacles were inflated with help of costmap node parameters.
- A* algorithm was developed using rospy and numpy library only.
- Proportional controller used odometry data to give commands for the motion of turtlebot on the obtained path.

Map with obtained path displayed in rviz:

	Start location : (0,0)
	Goal location : (-3,-4)

![](src/my_turtlebot/images/rviz_path.png)

**Result**

![](src/my_turtlebot/images/motion.gif)



 


