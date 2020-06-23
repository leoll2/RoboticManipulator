function targetPoseCallback(~, msg_pose)

    fprintf("targetPoseCallback invoked\n");
    global q_des
    global T_des
    global crane

    w = msg_pose.Orientation.W;
    x = msg_pose.Orientation.X;
    y = msg_pose.Orientation.Y;
    z = msg_pose.Orientation.Z;
    q = Quaternion([w x y z]);
    rot_q = UnitQuaternion(q.unit);
    tr = transl(msg_pose.Position.X, msg_pose.Position.Y, msg_pose.Position.Z);
    
    % Update the target joint position (performing inverse kinematics)
    T_des = tr * rot_q.T;
    q_des = crane.ikcon(T_des);
    % TODO here it is possible to see if the position is reachable (e.g.
    % crane.islimit(q_des)
end