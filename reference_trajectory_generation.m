% Sai Satya Charan Malladi
% Fall - 2021
% Final Project

clc; clear all; close all;

%% Begin 

% rocket parameters
g = 9.8;
m = 100000;
max_thrust = 2200*1000;
l = 50;
I = 1/12*m*l^2;

% pack everything
params.g = g;
params.m = m;
params.max_thrust = max_thrust;
params.l = l;
params.I = I;

% control intervals
N = 400;
opti = casadi.Opti();

% decision variables

% state
X = opti.variable(6,N+1);
x = X(1,:);
y = X(2,:);
theta = X(3,:);
xdot = X(4,:);
ydot = X(5,:);
thetadot = X(6,:);

% control 
U = opti.variable(2,N);
thrust_ratio = U(1,:);
thrust_angle = U(2,:);

% Time
T = 16;

% objective
opti.minimize(sumsqr(thrust_ratio(:)) + sumsqr(thrust_angle(:)) + sumsqr(theta(:)));

% 
dt = T/N;

% dynamics
f = @(X,U) rocket_dynamics(X,U,params);

% set dynamics contraints
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4);
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% set bounds on control
d2r = pi/180;
opti.subject_to(0.2 <= thrust_ratio <=1);
opti.subject_to(-20*d2r <= thrust_angle <= 20*d2r);
opti.subject_to(X(:,1) == [0; 1000; -pi/2; 0; -80; 0]);
opti.subject_to(X(:,N+1) == [0;0;0;0;0;0]);

% solve for the optimal trajectory
opti.solver('ipopt');
sol = opti.solve();


%s create the reference traj
ref_traj = [sol.value(x)', sol.value(y)',  sol.value(theta)', ...
            sol.value(xdot)', sol.value(ydot)', sol.value(thetadot)', ...
            (0:T/N:T)'];
ref_ctrl = [sol.value(thrust_ratio)', sol.value(thrust_angle)'];
aug = [zeros(1,6),T+4];
ref_traj = [ref_traj; aug]; 
save('reference_trajectory.mat','ref_traj')

% plot
plots_reference_trajectory;