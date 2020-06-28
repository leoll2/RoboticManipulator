%% Controller

function [] = crane_control_loop(robot, do_plot, do_print, do_publish)
    
    clear_figures;
    global opmode;
    global q;           % current joint position
    global q_des;       % desired joint position
    global T_des;       % desired end-effector-pose (redundant, only for convenience)
    global ee_pose_pub; % end-effector ROS publisher
    global end_eff_pos; % current position (x,y,z) of the end-effector
    global placed_objs; 

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
    Kp = diag([15 15 8 10 10 10]);
    Kv = diag([0.15 0.15 0.1 0.1 0.1 0.1]);
    Ki = diag([0 0 0 0 0 0]);
    
    % Controller limits (saturation)
    ctrl_max = [5 1 1 100 100 100];
    ctrl_min = -ctrl_max;
    
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
    drawn_objs = 0;
    
    figure(2)
    title('Joint #1 position');
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    q1_line = animatedline('Color', 'b');
    q1_ref_line = animatedline('Color', 'r');
    figure(3)
    title('Joint #2 position');
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    q2_line = animatedline('Color', 'b');
    q2_ref_line = animatedline('Color', 'r');
    figure(4)
    title('Joint #3 position');
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    q3_line = animatedline('Color', 'b');
    q3_ref_line = animatedline('Color', 'r');
    figure(5)
    title('Joint #1 torque');
    xlabel('Time [s]');
    ylabel('Torque [Nm]');
    tau1_ctrl_line = animatedline('Color', 'r');
    % out1_ctrl_line = animatedline('Color', 'r');
    figure(6)
    title('Joint #2 torque');
    xlabel('Time [s]');
    ylabel('Torque [Nm]');
    tau2_ctrl_line = animatedline('Color', 'r');
    % out2_ctrl_line = animatedline('Color', 'r');
    figure(7)
    title('Joint #3 torque');
    xlabel('Time [s]');
    ylabel('Torque [Nm]');
    tau3_ctrl_line = animatedline('Color', 'r');
    % out3_ctrl_line = animatedline('Color', 'r');
    
    figure(1)
    robot.plot(q, 'floorlevel', 0, 'nojoints');

    % Control loop
    while true

        % Errors
        err_old = err;
        err = q_des - q;
        derr = dq_des - dq;
        ierr = ierr + (err + err_old) * delta_t / 2;
        pos_des = transl(T_des)';

        %Get dynamic matrices
        G = robot.gravload(q);
        C = robot.coriolis(q, dq);
        M = robot.inertia(q);

        % Controller (Computed Torque)
        % tau_robot = robot.rne(q, dq, ddq);
        ctrl_out = Kp*err' + Kv/delta_t*derr' + Ki*delta_t*ierr';
        ctrl_out = clampv(ctrl_out, ctrl_min, ctrl_max);
        tau_ctrl = (M*(ctrl_out) + (C*dq') + G')';

        % System dynamics
        ddq_old = ddq;
        ddq = (pinv(M) * (tau_ctrl' - (C*dq') - G'))';

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
            fprintf('---------------------\n');
            fprintf('Time: %.3f     Opmode: %s\n', time, opmode);
            fprintf('q:     [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', q(1), q(2), q(3), q(4), q(5), q(6));
            fprintf('q_des: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', q_des(1), q_des(2), q_des(3), q_des(4), q_des(5), q_des(6));
            fprintf('x,y,z: [%.3f, %.3f, %.3f]\n', end_eff_pos(1), end_eff_pos(2), end_eff_pos(3));
        end
        
        % Plot the system in 3D
        if do_plot
            
            % Draw the robot
            robot.plot(q, 'floorlevel', 0, 'nojoints');
            
            % Undraw the object attached to the end effector, if any
            if exist('prev_ee_cube_s','var') == 1
               delete(prev_ee_cube_s);
               delete(prev_ee_cube_p);
            end
            
            hold on;
            % Draw the object attached to the end effector ('place' mode)
            % or the target object ('pick' mode)
            if strcmp(opmode, 'place')
                [prev_ee_cube_s, prev_ee_cube_p] = plotcube(end_eff_pos, [0.15 0.15 0.15]);
            elseif strcmp(opmode, 'pick')
                [prev_ee_cube_s, prev_ee_cube_p] = plotcube(pos_des, [0.15 0.15 0.15]);
            end

            for i = (drawn_objs+1):size(placed_objs, 1)
                fprintf("Index: %d\n", i);
                plotcube(placed_objs(i,:), [0.15 0.15 0.15]);
                drawn_objs = drawn_objs + 1;
            end
            hold off;
            
            % Plot joint positions over time
            addpoints(q1_line, time, mod(q(1), 2*pi));
            addpoints(q1_ref_line, time, mod(q_des(1), 2*pi));
            addpoints(q2_line, time, q(2));
            addpoints(q2_ref_line, time, q_des(2));
            addpoints(q3_line, time, q(3));
            addpoints(q3_ref_line, time, q_des(3));
            addpoints(tau1_ctrl_line, time, tau_ctrl(1));
            addpoints(tau2_ctrl_line, time, tau_ctrl(2));
            addpoints(tau3_ctrl_line, time, tau_ctrl(3));
            % addpoints(out1_ctrl_line, time, ctrl_out(1));
            % addpoints(out2_ctrl_line, time, ctrl_out(2));
            % addpoints(out3_ctrl_line, time, ctrl_out(3));
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
    end
end
