Trajectory planning integrated with ROS

# Run the code
1. build fanuc package in catkin
2. run launch file trajectory_move.launch (roslaunch fanuc trajectory_move.launch)
3. It will start trajectory planning node, RViz visualization and rqt plot for joint velocities
4. When it start input any symbol to terminal (correspondent message should appear in console)
5. After that test script for sequence of PTP, LIN, PTP, LIN commands will start
