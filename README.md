# ros-sync-messages
use approximate time to sync camera image messages with odometry messages


To run this node: cd into the package directory, catkin build (assuming you're using catkin_tools), cd into the directory you want the files to be saved into (I know, this should be a parameter to the node but that'll be future work hehe), run roscore in another terminal, run the node (rosrun sync_messages sync_messages), replay bag file in another terminal (rosbag play <.bag file>); When done, ctr+c the node, then roscore.
