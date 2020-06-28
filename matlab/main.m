clear
clear_figures;
clc

global q;               % joints position
global q0;              % initial joints position
global q_des;           % desired joints position
global T_des;           % desired pose (homogeneous transformation matrix)
global opmode;          % operating mode (rest/pick/place)
global crane;           % robot
global placed_objs;     % list of picked-and-placed objects

placed_objs = [];

crane = crane_model();
opmode = 'rest';
q0 = [pi, pi/4, -pi/2, pi/2, pi/2, -pi/2];
q = q0;
q_des = q;
T_des = crane.fkine(q_des);
crane.plot(q, 'floorlevel', 0);

% Run an animation showing the possible robot movements
% show_possible_movements(crane, q0);

ros_adaptor;

% Robot control loop (params: robot, do_plot, do_print, do_publish)
crane_control_loop(crane, true, true, true);