%% Controller

function [] = crane_control(robot, q0, q1, do_plot, do_print)

    if nargin < 4
        do_plot = false;
    end
    if nargin < 5
        do_print = false;
    end

    % Controller gain matrix
    Kp = diag([10 30 7 10 10 10]);
    Kv = diag([0.15 0.25 0.1 0.1 0.1 0.1]);
    Ki = diag([0 0 0 0 0 0]);
    
    % Time
    t_in = 0; % [s]
    t_fin = 10; % [s]
    delta_t = 0.02; % [s]
    t = t_in:delta_t:t_fin;
    
    % Joints
    n_joints = length(q0);
    
    % Initial conditions
    q = q0;
    dq = zeros(1, n_joints);
    ddq = zeros(1, n_joints);
    qi = zeros(1, n_joints);

    % References of position, velocity and acceleration of the joints 
    q_des = q1;                         % desired joint position
    dq_des = zeros(1, n_joints);        % desired joint speed
    ddq_des = zeros(1, n_joints);       % desired joint acceleration
    qi_des = zeros(1, n_joints);        % TODO remove
    err_old = q_des - q;                % previous error
    err = q_des - q;                    % error
    ierr = zeros(1, n_joints);          % integral of error
    
    result = zeros(length(t), n_joints);
    index = 1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Computed Torque               %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    for i=1:length(t)

        % Errors
        err_old = err;
        err = q_des - q;
        derr = dq_des - dq;
        ierr = ierr + (err + err_old) * delta_t / 2;

        %Get dynamic matrices
        G = robot.gravload(q);
        C = robot.coriolis(q, dq);
        M = robot.inertia(q);

        % Controller
        tau_robot = robot.rne(q, dq, ddq);
        tau_ext = (M*(Kp*err' + Kv/delta_t*derr' + Ki*delta_t*ierr') + (C*dq') + G')';
        % tau = tau_robot + tau_ext;

        % System dynamics
        ddq_old = ddq;
        ddq = (pinv(M) * (tau_ext' - (C*dq') - G'))';
        %ddq = (pinv(M)*(tau - (C*dq')' - G)')';

        % Robot joint position and speed
        dq_old = dq;
        dq = dq + (ddq_old + ddq) * delta_t / 2;
        q_old = q;
        q = q + (dq + dq_old) * delta_t /2;
        
        T = robot.fkine(q);
        end_eff_pos = transl(T);
       
        % Store result for the final plot
        result(index,:) = q;
        index = index + 1;

        if do_print
            fprintf('---------------------\ntime: %.3f\n', i*delta_t);
            fprintf('err0:   %.3f   derr0:  %.3f    ierr0:  %.3f\n', err(1), derr(1), ierr(1));
            %fprintf('tauR0:  %.3f\n', tau_robot(1));
            %fprintf('tauP0:  %.3f   tauV0:  %.3f    tauI0:  %.3f\n', Kp(1,1)*err(1), Kv(1,1)/delta_t*derr(1), Ki(1,1)*delta_t*ierr(1));
            %fprintf('tauR1:  %.3f\n', tau_robot(2));
            %fprintf('tauP1:  %.3f   tauV1:  %.3f    tauI1:  %.3f\n', Kp(2,2)*err(2), Kv(2,2)/delta_t*derr(2), Ki(2,2)*delta_t*ierr(2));
            %fprintf('tauR2:  %.3f\n', tau_robot(3));
            %fprintf('tauP2:  %.3f   tauV2:  %.3f    tauI2:  %.3f\n', Kp(3,3)*err(3), Kv(3,3)/delta_t*derr(3), Ki(3,3)*delta_t*ierr(3));
        end
        
        if do_plot
            robot.plot(q, 'floorlevel', 0);
            plotcube(end_eff_pos, [0.24 0.24 0.24]);
        end
    end
end
