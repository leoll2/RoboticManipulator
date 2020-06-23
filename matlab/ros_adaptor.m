% Start ROS master
% rosshutdown()
% rosinit('localhost',11311)

% Show nodes
fprintf("Nodes: \n");
rosnode list

% Show topics
fprintf("Topics: \n");
rostopic list

% Subscribers
global target_pose_sub
target_pose_sub = rossubscriber('/end_effector/target_pose', 'geometry_msgs/Pose', @targetPoseCallback);

% Publishers
global ee_pose_pub
ee_pose_pub = rospublisher('/end_effector/pose', 'geometry_msgs/Pose');
