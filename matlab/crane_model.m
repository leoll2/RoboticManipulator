clear
close all
clc

%% Parameters
% lenghts [dm]
j1_z = 0.2;   % height of the joint between mast and boom
j1_x = -0.1;  % horizontal offset of the join between mast and boom
boom_l = 0.3; % length of the boom 
jib_l = 0.5;  % length of the jib


%% Denavit-Hartenberg
% Axes:
%   z: rotation axis (revolute) or translation (prismatic)
%   x: collinear with common normal between the z axis of this and the previous joint.
%      If this is the first joint, choose any direction
%   y: complete the right-handed
%   O: where the normal intersects the z-axis
%
% DH parameters:
%   d:      depth along previous joint z-axis (from prev origin to common normal)
%   theta:  rot angle around prev z-axis to align old x-axis to the new one
%   a:      length of the common normal (distance along the rotated x-axis)
%           It is also the radius of revolution about previous z-axis
%   alpha:  rot angle about the new x-axis to align old z-axis to the new one
%
% Robotics Toolbox Link parameters order:
%   theta, d, a, alpha

% Standard DH
L1 = Link([0, j1_z, j1_x, pi/2]);    % Mast
L2 = Link([0, 0, boom_l, 0]);        % Boom
L3 = Link([0, 0, jib_l, 0]);         % Jib
L4 = Link([0, 0, 0, -pi/2]);         % Wrist
L5 = Link([0, 0, 0, pi/2]);          % Wrist
L6 = Link([0, 0, 0, 0]);             % Wrist

%% Joint limits
% angles [rad]
L1.qlim = [];
L2.qlim = [pi/6, 2*pi/3];
L3.qlim = [-2*pi/3, 0];

%% Link masses
% mass [kg]
L1.m = 8;
L2.m = 9;
L3.m = 15;
L4.m = 0.3;
L5.m = 0.3;
L6.m = 0.3;

%% Inertia matrix
% https://amesweb.info/inertia/mass-moment-of-inertia-calculator.aspx
L1.I = [
    0.03  0     0;
    0     0.03  0;
    0     0     0.006;
];
L2.I = [
    0.06  0     0;
    0     0.06  0;
    0     0     0.006;
];
L3.I = [
    0.16  0     0;
    0     0.16  0;
    0     0     0.006;
];
L4.I = [
    0.001  0     0;
    0     0.001  0;
    0     0      0.001;
];
L5.I = [
    0.001  0     0;
    0     0.001  0;
    0     0      0.001;
];
L6.I = [
    0.001  0     0;
    0     0.001  0;
    0     0      0.001;
];

% Ignore motor inertia (for simplicity)
L1.Jm = 0;
L2.Jm = 0;
L3.Jm = 0;
L4.Jm = 0;
L5.Jm = 0;
L6.Jm = 0;

% Instantiate crane and set initial position
crane = SerialLink([L1, L2, L3, L4, L5, L6], 'name', 'crane');
q0 = [0, pi/4, -pi/2, 0, 0, 0];
q = q0;
crane.plot(q, 'floorlevel', 0)

% Run an animation showing the possible robot movements
%show_possible_movements(crane, q0);

% Target position
hold on
%T = transl(0., 0., 0.5); %* rpy2tr(0, 0, 180, 'deg');
%trplot(T);

% Inverse kinematics to find target joint configuration
%q_des = crane.ikcon(T);
%crane.plot(q_des, 'floorlevel', 0)

%q_des = [pi/2, pi/4, -pi/2, 0, 0, 0];
%q_des = [0, pi/2, -pi/2, 0, 0, 0];
%q_des = [0, pi/4, -pi/6, 0, 0, 0];
%q_des = [0, pi/4, -pi/2, pi/4, pi/4, pi/4];
q_des = [-pi/2, pi/2, -pi/4, pi/4, pi/4, -pi/4];
T = crane.fkine(q_des);
trplot(T);
% Move to desired position
crane_control(crane, q, q_des, true, true);

