clear
close all
clc

global q;
global q_des;
global T_des;
global crane;

crane = crane_model();

q0 = [0, pi/4, -pi/2, 0, 0, 0];
q = q0;
q_des = q;
T_des = crane.fkine(q_des);
crane.plot(q, 'floorlevel', 0)

% Run an animation showing the possible robot movements
% show_possible_movements(crane, q0);

ros_adaptor;

% Robot control loop (params: robot, do_plot, do_print, do_publish)
crane_control_loop(crane, true, true, true);

% % Move to desired position
% crane_control(crane, q, q_des, true, true);