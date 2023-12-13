% Sai Satya Charan Malladi
% AEROSP 740 - Fall 2021
% Final Project

% clc; clear all; close all;

%% Begin 

% generate a reference trajectory
reference_trajectory_generation;

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

% define control inputs and outputs 
nx = 6;
ny = 6;
nu = 2;

% create the nlmpc object
nlobj = nlmpc(nx,ny,nu);

% model dynamics and jacobian 
nlobj.Model.StateFcn = @(x,u) rocket_dynamics(x,u,params);
nlobj.Jacobian.StateFcn = @(x,u) rocket_dynamics_jacobian(x,u,params);

% validate
rng(0)
validateFcns(nlobj,rand(nx,1),rand(nu,1));

% 
Ts = 0.1;
p = 30;     % prediction horizon 
m = 10;      % control horizon
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = m;

% limits on state and control variables
d2r = pi/180;
r2d = 180/pi;
nlobj.MV = struct('Min',{0;-20*d2r},'Max',{1;20*d2r});
nlobj.States = struct('Min',{-inf;0;-pi/2;-inf;-inf;-inf},'Max',{inf;inf;pi/2;inf;inf;inf});

% check weights for all variables
Q = [10 10 10 0 0 0];
R = [0.1 0.1];
nlobj.Weights.OutputVariables = Q;
nlobj.Weights.ManipulatedVariables = [0 0];
nlobj.Weights.ManipulatedVariablesRate = R;

% 
Duration = 16; % 16 secs
time = 0:Ts:Duration;

% three different cases
% should run many more to qunatify stability
for case_num = 3

    switch case_num
        % different cases: model-plant mismatch
        case 1
            % initial conditions
            x = [0; 1000; -pi/2; 0; -80; 0];
            mv = [0 0];
            fig_labels = {'case 1: $X_{0}$= [10 1000 $-\frac{\pi}{2}$ 0 -80 0]', ...
                        'case_1_state_control', ...
                        'case 1: trajectory', ...
                        'case_1_traj'};
        case 2
            % initial conditions
            x = [10; 1000; -pi/2; 0; -90; 0];
            mv = [0 0];
            fig_labels = {'case 2: $X_{0}$= [10 1000 $-\frac{\pi}{2}$ 0 -90 0]', ...
                        'case_2_state_control', ...
                        'case 2: trajectory', ...
                        'case_2_traj'};

        case 3
            % initial conditions
            x = [10; 1000; -pi/2; 0; -90; 0];
            mv = [0 0];
            fig_labels = {'case 3: $X_{0}$= [10 1000 $-\frac{\pi}{2}$ 0 -90 0]', ...
                        'case_3_state_control', ...
                        'case 3: trajectory', ...
                        'case_3_traj'};

            % change m for simulations
            params.m = 95000;
    end
    
    nloptions = nlmpcmoveopt;

    % check yref
    load('reference_trajectory.mat')
    xHistory = x';
    lastMV = mv;
    uHistory = lastMV;
    for k = 1:(Duration/Ts)
        % Set references for previewing
        t = linspace(k*Ts, (k+p-1)*Ts,p);
        yref = interp1(ref_traj(:,end), ref_traj(:,1:6), t)';

        % Compute the control moves with reference previewing.
        xk = xHistory(k,:);
        [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
        uHistory(k+1,:) = uk';
        lastMV = uk;

        % Update states.
        ODEFUN = @(t,xk) rocket_dynamics(xk,uk,params);
        [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
        xHistory(k+1,:) = YOUT(end,:);
    end

    % plot and animate
    plots_mpc;
    animate;

end