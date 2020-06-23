%% Controller

function [] = crane_control_loop(robot, do_plot, do_print, do_publish)

    global q;           % current joint position
    global q_des;       % desired joint position
    global T_des;       % desired end-effector-pose (redundant, only for convenience)
    global ee_pose_pub; % end-effector ROS publisher

    if nargin < 2
        do_plot = false;
    end
    if nargin < 3
        do_print = false;
    end
    if nargin < 4
        do_publish = false;
    end

    % Controller gain matrix
    Kp = diag([15 30 7 10 10 10]);
    Kv = diag([0.15 0.25 0.1 0.1 0.1 0.1]);
    Ki = diag([0 0 0 0 0 0]);
    
    % Time
    delta_t = 0.02;     % [s] controller period
    
    % Joints
    n_joints = length(q);
    
    % Initial conditions
    dq = zeros(1, n_joints);
    ddq = zeros(1, n_joints);

    % References of position, velocity and acceleration of the joints                     
    dq_des = zeros(1, n_joints);        % desired joint speed
    ddq_des = zeros(1, n_joints);       % desired joint acceleration
    err = q_des - q;                    % error
    ierr = zeros(1, n_joints);          % integral of error
    
    iteration = 1;

    % Control loop
    while true

        % Errors
        err_old = err;
        err = q_des - q;
        derr = dq_des - dq;
        ierr = ierr + (err + err_old) * delta_t / 2;

        %Get dynamic matrices
        G = robot.gravload(q);
        C = robot.coriolis(q, dq);
        M = robot.inertia(q);

        % Controller (Computed Torque)
        % tau_robot = robot.rne(q, dq, ddq);
        tau_ext = (M*(Kp*err' + Kv/delta_t*derr' + Ki*delta_t*ierr') + (C*dq') + G')';

        % System dynamics
        ddq_old = ddq;
        ddq = (pinv(M) * (tau_ext' - (C*dq') - G'))';

        % Robot joint position and speed
        dq_old = dq;
        dq = dq + (ddq_old + ddq) * delta_t / 2;
        q = q + (dq + dq_old) * delta_t /2;
        
        % End effector pose
        T = robot.fkine(q);
        [end_eff_rot, end_eff_pos] = tr2rt(T);
       
        % Increment iteration counter
        iteration = iteration + 1;
        time = iteration * delta_t;

        % Print info
        if do_print
            fprintf('---------------------\ntime: %.3f\n', time);
            fprintf('q:     [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', q(1), q(2), q(3), q(4), q(5), q(6));
            fprintf('q_des: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', q_des(1), q_des(2), q_des(3), q_des(4), q_des(5), q_des(6));
            fprintf('x,y,z: [%.3f, %.3f, %.3f]\n', end_eff_pos(1), end_eff_pos(2), end_eff_pos(3));
            %fprintf('err0:   %.3f   derr0:  %.3f    ierr0:  %.3f\n', err(1), derr(1), ierr(1));
            %fprintf('tauR0:  %.3f\n', tau_robot(1));
            %fprintf('tauP0:  %.3f   tauV0:  %.3f    tauI0:  %.3f\n', Kp(1,1)*err(1), Kv(1,1)/delta_t*derr(1), Ki(1,1)*delta_t*ierr(1));
            %fprintf('tauR1:  %.3f\n', tau_robot(2));
            %fprintf('tauP1:  %.3f   tauV1:  %.3f    tauI1:  %.3f\n', Kp(2,2)*err(2), Kv(2,2)/delta_t*derr(2), Ki(2,2)*delta_t*ierr(2));
            %fprintf('tauR2:  %.3f\n', tau_robot(3));
            %fprintf('tauP2:  %.3f   tauV2:  %.3f    tauI2:  %.3f\n', Kp(3,3)*err(3), Kv(3,3)/delta_t*derr(3), Ki(3,3)*delta_t*ierr(3));
        end
        
        % Plot robot
        if do_plot
            robot.plot(q, 'floorlevel', 0);
            %hold on;
            trplot(T_des);
            hold off;
            % plotcube(end_eff_pos, [0.24 0.24 0.24]);
        end
        
        % Publish
        if do_publish
            pub_msg = rosmessage('geometry_msgs/Pose');
            pub_msg.Position.X = end_eff_pos(1);
            pub_msg.Position.Y = end_eff_pos(2);
            pub_msg.Position.Z = end_eff_pos(3);
            ee_rot_uq = UnitQuaternion(end_eff_rot);
            pub_msg.Orientation.X = ee_rot_uq.v(1);
            pub_msg.Orientation.Y = ee_rot_uq.v(2);
            pub_msg.Orientation.Z = ee_rot_uq.v(3);
            pub_msg.Orientation.W = ee_rot_uq.s;
            send(ee_pose_pub, pub_msg);
        end
        
%         if time > 0.6
%             q_des = [-pi/2, pi/2, -pi/4, pi/4, pi/4, -pi/4];
%         end
    end
end
