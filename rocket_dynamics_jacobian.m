function [A,B] = rocket_dynamics_jacobian(X,U,params)


% states 
x = X(1,:);
y = X(2,:);
theta = X(3,:);
xdot = X(4,:);
ydot = X(5,:);
thetadot = X(6,:);

% unpack params
g = params.g;
m = params.m;
max_thrust = params.max_thrust;
l = params.l;
I = params.I;

% control
thrust_ratio = U(1,:);
thrust_angle = U(2,:);


A = [0 0 0 1 0 0;...
     0 0 0 0 1 0;...
     0 0 0 0 0 1;...
     0 0 (max_thrust*thrust_ratio*cos(theta + thrust_angle))/m 0 0 0;
     0 0 -(max_thrust*thrust_ratio*sin(theta + thrust_angle))/m 0 0 0;
     0 0 0 0 0 0];
 
B = [                                              0,                                                                 0;...
                                                   0,                                                                 0;...
                                                   0,                                                                 0;...
            (max_thrust*sin(theta + thrust_angle))/m,             (max_thrust*thrust_ratio*cos(theta + thrust_angle))/m;...
            (max_thrust*cos(theta + thrust_angle))/m,            -(max_thrust*thrust_ratio*sin(theta + thrust_angle))/m;...
             -(l*max_thrust*sin(thrust_angle))/(2*I),            -(l*max_thrust*thrust_ratio*cos(thrust_angle))/(2*I)];

end