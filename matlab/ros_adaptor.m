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
target_pose_sub = rossubscriber('robot/end_effector/target_pose', 'geometry_msgs/Pose', @targetPoseCallback);
global op_mode_sub
op_mode_sub = rossubscriber('robot/opmode', 'std_msgs/String', @opmodeCallback);


% Publishers
global ee_pose_pub
ee_pose_pub = rospublisher('robot/end_effector/pose', 'geometry_msgs/Pose');
