% Sai Satya Charan Malladi
% AEROSP 740 - Fall 2021
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

% define control inputs and outputs 
nx = 6;
ny = 6;
nu =2;

% create the nlmpc object
nlobj = nlmpc(nx,ny,nu);

% model dynamics and jacobian 
nlobj.Model.StateFcn = @(x,u) rocketDynamics(x,u,params);
nlobj.Jacobian.StateFcn = @(x,u) rocketDynamicsJacobian(x,u,params);

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


%% Case - 1

% Specify the initial conditions
x = [0; 1000; -pi/2; 0; -80; 0];
mv = [0 0];
nloptions = nlmpcmoveopt;

% Test for Robustness
% change m for simulations
% params.m = 95000;

% check yref
load('ref_traj_new.mat')
Duration = 16; % 16 secs
xHistory = x';
lastMV = mv;
uHistory = lastMV;
for k = 1:(Duration/Ts)
    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = interp1(ref_traj(:,end), ref_traj(:,1:6), t)';
%     yref(4:end,:) = zeros(size(yref(4:end,:)));
    
    % Compute the control moves with reference previewing.
    xk = xHistory(k,:);
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
    uHistory(k+1,:) = uk';
    lastMV = uk;
    
    % Update states.
    ODEFUN = @(t,xk) rocketDynamics(xk,uk,params);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    xHistory(k+1,:) = YOUT(end,:);
end


%% plots  
fig1 = figure('Position', get(0, 'Screensize'));
fig1tile = tiledlayout(4,2,'TileSpacing','tight','Padding','tight');

time = 0:Ts:Duration;

% subplot(4,2,1)
nexttile
hold on
plot(time,xHistory(:,1),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,1),'k--','LineWidth',2,'DisplayName','Reference')
xlim([0 16])
hold off
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$x$ [m]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(ref_traj(:,7),ref_traj(:,1),'k-','LineWidth',2,'DisplayName','Reference')
plot(time,xHistory(:,4),'r-','LineWidth',2,'DisplayName','MPC')
xlim([0 16])
hold off
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{x}$ [m/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,2),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,2),'k--','LineWidth',2,'DisplayName','Reference')
yline(0,'b--','LineWidth',2,'DisplayName','constraint')
yline(1000,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$y$ [m]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(ref_traj(:,7),ref_traj(:,2),'k-','LineWidth',2,'DisplayName','Reference')
plot(time,xHistory(:,5),'r-','LineWidth',2,'DisplayName','MPC')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{y}$ [m/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,3)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,3)*r2d,'k--','LineWidth',2,'DisplayName','Reference')
yline(90,'b--','LineWidth',2,'DisplayName','constraint')
yline(-90,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\theta$ [deg]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(ref_traj(:,7),ref_traj(:,3),'k-','LineWidth',2,'DisplayName','Reference')
plot(time,xHistory(:,6)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{\theta}$ [deg/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on


nexttile
hold on
stairs(time,uHistory(:,1),'r-','LineWidth',2,'DisplayName','MPC')
yline(1,'b--','LineWidth',2,'DisplayName','constraint')
yline(0,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('Thrust ratio $\gamma$','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
stairs(time,uHistory(:,2)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
yline(-20,'b--','LineWidth',2,'DisplayName','constraint')
yline(20,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('Thrust angle, $\delta$ [deg]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on
sgtitle(['Case 1: $X_{0}$= [10 1000 $-\frac{\pi}{2}$ 0 -80 0]' ],'fontsize',30,'interpreter','latex');
saveas(fig1,'fig1','epsc')

fig1i = figure('Position', get(0, 'Screensize'));
hold on
xlim([-150 150])
ylim([0 1000])
plot(xHistory(:,1),xHistory(:,2),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,1),ref_traj(:,2),'k--','LineWidth',2,'DisplayName','Reference')
hold off
set(gca,'FontSize',30)
set(gca,'TickLabelInterpreter','latex');
xlabel('$x$ [m]','fontsize',30,'interpreter','latex')
ylabel('$y$ [m]','fontsize',30,'interpreter','latex')
legend('Location','best','fontsize',30,'interpreter','latex')
title('Case 1: Trajectory','fontsize',30,'interpreter','latex')
grid on
axis equal
saveas(fig1i,'fig1i','epsc')



%% Case - 2

% Specify the initial conditions
x = [10; 1000; -pi/2; 0; -90; 0];
mv = [0 0];
nloptions = nlmpcmoveopt;

% Test for Robustness
% change m for simulations
% params.m = 95000;

% check yref
load('ref_traj_new.mat')
Duration = 16; % 16 secs
xHistory = x';
lastMV = mv;
uHistory = lastMV;
for k = 1:(Duration/Ts)
    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = interp1(ref_traj(:,end), ref_traj(:,1:6), t)';
%     yref(4:end,:) = zeros(size(yref(4:end,:)));
    
    % Compute the control moves with reference previewing.
    xk = xHistory(k,:);
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
    uHistory(k+1,:) = uk';
    lastMV = uk;
    
    % Update states.
    ODEFUN = @(t,xk) rocketDynamics(xk,uk,params);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    xHistory(k+1,:) = YOUT(end,:);
end

%%
fig2 = figure('Position', get(0, 'Screensize'));
fig2tile = tiledlayout(4,2,'TileSpacing','tight','Padding','tight');

time = 0:Ts:Duration;

% subplot(4,2,1)
nexttile
hold on
plot(time,xHistory(:,1),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,1),'k--','LineWidth',2,'DisplayName','Reference')
xlim([0 16])
hold off
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$x$ [m]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(ref_traj(:,7),ref_traj(:,1),'k-','LineWidth',2,'DisplayName','Reference')
plot(time,xHistory(:,4),'r-','LineWidth',2,'DisplayName','MPC')
xlim([0 16])
hold off
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{x}$ [m/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,2),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,2),'k--','LineWidth',2,'DisplayName','Reference')
yline(1000,'b--','LineWidth',2,'HandleVisibility','off')
yline(0,'b--','LineWidth',2,'DisplayName','constraint')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$y$ [m]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(ref_traj(:,7),ref_traj(:,2),'k-','LineWidth',2,'DisplayName','Reference')
plot(time,xHistory(:,5),'r-','LineWidth',2,'DisplayName','MPC')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{y}$ [m/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,3)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,3)*r2d,'k--','LineWidth',2,'DisplayName','Reference')
yline(90,'b--','LineWidth',2,'DisplayName','constraint')
yline(-90,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\theta$ [deg]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(ref_traj(:,7),ref_traj(:,3),'k-','LineWidth',2,'DisplayName','Reference')
plot(time,xHistory(:,6)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{\theta}$ [deg/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on


nexttile
hold on
stairs(time,uHistory(:,1),'r-','LineWidth',2,'DisplayName','MPC')
yline(1,'b--','LineWidth',2,'DisplayName','constraint')
yline(0,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('Thrust ratio $\gamma$','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
stairs(time,uHistory(:,2)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
yline(-20,'b--','LineWidth',2,'DisplayName','constraint')
yline(20,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('Thrust angle, $\delta$ [deg]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on
sgtitle(['Case 2: $X_{0}$= [10 1000 $-\frac{\pi}{2}$ 0 -90 0]' ],'fontsize',30,'interpreter','latex');
saveas(fig2,'fig2','epsc')

fig2i = figure('Position', get(0, 'Screensize'));
hold on
xlim([-150 150])
ylim([0 1000])
plot(xHistory(:,1),xHistory(:,2),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,1),ref_traj(:,2),'k--','LineWidth',2,'DisplayName','Reference')
hold off
set(gca,'FontSize',30)
set(gca,'TickLabelInterpreter','latex');
xlabel('$x$ [m]','fontsize',30,'interpreter','latex')
ylabel('$y$ [m]','fontsize',30,'interpreter','latex')
legend('Location','best','fontsize',30,'interpreter','latex')
title('Case 2: Trajectory','fontsize',30,'interpreter','latex')
grid on
axis equal
saveas(fig2i,'fig2i','epsc')


%% case - 3

% Specify the initial conditions
x = [10; 1000; -pi/2; 0; -90; 0];
mv = [0 0];
nloptions = nlmpcmoveopt;

% Test for Robustness
% change m for simulations
params.m = 95000;

% check yref
load('ref_traj_new.mat')
Duration = 16; % 16 secs
xHistory = x';
lastMV = mv;
uHistory = lastMV;
for k = 1:(Duration/Ts)
    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = interp1(ref_traj(:,end), ref_traj(:,1:6), t)';
%     yref(4:end,:) = zeros(size(yref(4:end,:)));
    
    % Compute the control moves with reference previewing.
    xk = xHistory(k,:);
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
    uHistory(k+1,:) = uk';
    lastMV = uk;
    
    % Update states.
    ODEFUN = @(t,xk) rocketDynamics(xk,uk,params);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    xHistory(k+1,:) = YOUT(end,:);
end

%% 
fig3 = figure('Position', get(0, 'Screensize'));
fig3tile = tiledlayout(4,2,'TileSpacing','tight','Padding','tight');

time = 0:Ts:Duration;

% subplot(4,2,1)
nexttile
hold on
plot(time,xHistory(:,1),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,1),'k--','LineWidth',2,'DisplayName','Reference')
xlim([0 16])
hold off
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$x$ [m]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(ref_traj(:,7),ref_traj(:,1),'k-','LineWidth',2,'DisplayName','Reference')
plot(time,xHistory(:,4),'r-','LineWidth',2,'DisplayName','MPC')
xlim([0 16])
hold off
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{x}$ [m/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,2),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,2),'k--','LineWidth',2,'DisplayName','Reference')
yline(0,'b--','LineWidth',2,'DisplayName','constraint')
yline(1000,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$y$ [m]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(ref_traj(:,7),ref_traj(:,2),'k-','LineWidth',2,'DisplayName','Reference')
plot(time,xHistory(:,5),'r-','LineWidth',2,'DisplayName','MPC')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{y}$ [m/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,3)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,3)*r2d,'k--','LineWidth',2,'DisplayName','Reference')
yline(90,'b--','LineWidth',2,'DisplayName','constraint')
yline(-90,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\theta$ [deg]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(ref_traj(:,7),ref_traj(:,3),'k-','LineWidth',2,'DisplayName','Reference')
plot(time,xHistory(:,6)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{\theta}$ [deg/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on


nexttile
hold on
stairs(time,uHistory(:,1),'r-','LineWidth',2,'DisplayName','MPC')
yline(1,'b--','LineWidth',2,'DisplayName','constraint')
yline(0,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('Thrust ratio $\gamma$','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
stairs(time,uHistory(:,2)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
yline(-20,'b--','LineWidth',2,'DisplayName','constraint')
yline(20,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('Thrust angle, $\delta$ [deg]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on
sgtitle(['Case 3: $X_{0}$= [10 1000 $-\frac{\pi}{2}$ 0 -90 0] and $m = $ 95 tonnes(ode45 integrator)' ],'fontsize',30,'interpreter','latex');
saveas(fig3,'fig3','epsc')

fig3i = figure('Position', get(0, 'Screensize'));
hold on
xlim([-150 150])
ylim([0 1000])
plot(xHistory(:,1),xHistory(:,2),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,1),ref_traj(:,2),'k--','LineWidth',2,'DisplayName','Reference')
hold off
set(gca,'FontSize',30)
set(gca,'TickLabelInterpreter','latex');
xlabel('$x$ [m]','fontsize',30,'interpreter','latex')
ylabel('$y$ [m]','fontsize',30,'interpreter','latex')
legend('Location','best','fontsize',30,'interpreter','latex')
% legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
title('Case 3: Trajectory','fontsize',30,'interpreter','latex')
grid on
axis equal
saveas(fig3i,'fig3i','epsc')
