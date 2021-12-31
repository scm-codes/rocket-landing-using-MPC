function Xdot = rocketDynamics(X,U,params)
% dxdt = ode(x,u)
% rocket dynamics
%
% Inputs
% x - state
% u - control
%
% Outputs
% dxdt - derivative of state

% unpack params
g = params.g;
m = params.m;
max_thrust = params.max_thrust;
l = params.l;
I = params.I;

% 
theta = X(3,:);
xdot = X(4,:);
ydot = X(5,:);
thetadot = X(6,:);

% thrust ratio and deflection
thrust_ratio = U(1,:);
thrust_angle = U(2,:);


% nonlinear dynamics
xddot = max_thrust*thrust_ratio.*sin(theta+thrust_angle)/m;
yddot = max_thrust*thrust_ratio.*cos(theta+thrust_angle)/m - g;
thetaddot = -l/2*max_thrust*thrust_ratio.*sin(thrust_angle)/I;


Xdot = [xdot; ydot; thetadot; xddot; yddot; thetaddot];

end 