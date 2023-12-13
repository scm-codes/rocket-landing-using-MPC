% Code from the previous time to compute Ad and Bb
% x_{k+1} = Ad*x_{k} + Bd*u_{k}
clc; clear all; close all
syms x xdot y ydot theta thetadot thrust_angle thrust_ratio g m max_thrust l I

% g = 9.8;
% m = 100000;
% max_thrust = 2210*1000;
% l = 50;
% I = 1/12*m*l^2;

% nonlinear dynamics
xddot = max_thrust*thrust_ratio*sin(theta+thrust_angle)/m;
yddot = max_thrust*thrust_ratio*cos(theta+thrust_angle)/m - g;
thetaddot = -l/2*max_thrust*thrust_ratio*sin(thrust_angle)/I;

Xdot = [xdot; ydot; thetadot; xddot; yddot; thetaddot];

% computing A and B
A = jacobian(Xdot,[x y theta xdot ydot thetadot]);
B = jacobian(Xdot,[thrust_ratio thrust_angle]);

