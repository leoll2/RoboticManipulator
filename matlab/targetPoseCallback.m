function targetPoseCallback(~, msg_pose)

    global q
    global q_des
    global T_des
    global crane

    w = msg_pose.Orientation.W;
    x = msg_pose.Orientation.X;
    y = msg_pose.Orientation.Y;
    z = msg_pose.Orientation.Z;
    rot_q = Quaternion([w x y z]);
    rot_uq = UnitQuaternion(rot_q.unit);
    tr = transl(msg_pose.Position.X, msg_pose.Position.Y, msg_pose.Position.Z);
    
    % Update the target joint position (performing inverse kinematics)
    T_des = tr * rot_uq.T;
    q_des = crane.ikcon(T_des);
    
    % Make sure the robot dotates in the closest direction
    q(1) = mod(q(1), 2*pi);
    q_des(1) = mod(q_des(1), 2*pi);
    if (q_des(1) - q(1) > pi)
        q_des(1) = q_des(1) - 2*pi;
    elseif (q_des(1) - q(1) < -pi)
        q_des(1) = q_des(1) + 2*pi;
    end
    
    % Raise a warning if the desired position hits the joints limits
    if crane.islimit(q_des)
        fprintf('Warning: some joint is going to reach its limit; is the target pose reachable?');
    end
end
