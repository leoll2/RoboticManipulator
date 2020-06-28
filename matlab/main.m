clear
clear_figures;
clc

global q;
global q_des;
global T_des;
global opmode;
global crane;
global placed_objs;

placed_objs = [];

crane = crane_model();
opmode = 'idle';
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