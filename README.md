# Disaster_response_project
Autonomous navigation of a drone in a fire accident to detect human beings
The current version of code involves the drone iris with a LiDAR collecting data from 360 degrees and navigating through the given world to the aruco mark by avoiding obstacles.
Basic description of the algorithm:
The drone checks for distances using the LiDAR in four directions, and forms a path in the direction where there is more distance from obstacles.
It also stores the already visited nodes so that it doesn't go in the same path again.
When there is no unvisited node found within the threshold distance in all directions then exception is raised and the program ends.
Gazebo, ROS, Python, PX4 are used in the project.
