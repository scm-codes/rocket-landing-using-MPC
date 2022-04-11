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
f = @(X,U) rocketDynamics(X,U,params);

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


%% create the reference traj

ref_traj = [sol.value(x)', sol.value(y)',  sol.value(theta)', ...
            sol.value(xdot)', sol.value(ydot)', sol.value(thetadot)', ...
            (0:T/N:T)'];
ref_ctrl = [sol.value(thrust_ratio)', sol.value(thrust_angle)'];
aug = [zeros(1,6),T+4];
ref_traj = [ref_traj; aug]; 
save('ref_traj.mat','ref_traj')


%% plots  
figref = figure('Position', get(0, 'Screensize'));
figreftile = tiledlayout(4,2,'TileSpacing','tight','Padding','tight');

r2d = 180/pi;
% time = 0:Ts:Duration;

% subplot(4,2,1)
nexttile
hold on
% plot(time,xHistory(:,1),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,1),'k-','LineWidth',2,'DisplayName','optimal control')
xlim([0 16])
hold off
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$x$ [m]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
% legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(ref_traj(:,7),ref_traj(:,4),'k-','LineWidth',2,'DisplayName','optimal control')
% plot(time,xHistory(:,4),'r-','LineWidth',2,'DisplayName','MPC')
xlim([0 16])
hold off
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{x}$ [m/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
% legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(time,xHistory(:,2),'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,2),'k-','LineWidth',2,'DisplayName','optimal control')
yline(0,'b--','LineWidth',2,'DisplayName','constraint')
yline(1000,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$y$ [m]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
% legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(ref_traj(:,7),ref_traj(:,5),'k-','LineWidth',2,'DisplayName','optimal control')
% plot(time,xHistory(:,5),'r-','LineWidth',2,'DisplayName','MPC')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{y}$ [m/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
% legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
% plot(time,xHistory(:,3)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
plot(ref_traj(:,7),ref_traj(:,3)*r2d,'k-','LineWidth',2,'DisplayName','optimal control')
yline(90,'b--','LineWidth',2,'DisplayName','constraint')
yline(-90,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\theta$ [deg]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
% legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(ref_traj(:,7),ref_traj(:,6)*r2d,'k-','LineWidth',2,'DisplayName','optimal control')
% plot(time,xHistory(:,6)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{\theta}$ [deg/s]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
% legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on


nexttile
hold on
stairs(ref_traj(2:end-1,7),ref_ctrl(:,1),'k-','LineWidth',2,'DisplayName','optimal control')
yline(1,'b--','LineWidth',2,'DisplayName','constraint')
yline(0.2,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('Thrust ratio $\gamma$','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
% legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
stairs(ref_traj(2:end-1,7),ref_ctrl(:,2)*r2d,'k-','LineWidth',2,'DisplayName','optimal control')
yline(-20,'b--','LineWidth',2,'DisplayName','constraint')
yline(20,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('Thrust angle, $\delta$ [deg]','fontsize',20,'interpreter','latex')
% legend('Location','best','fontsize',15,'interpreter','latex')
% legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on
sgtitle(['Optimal Control Solution' ],'fontsize',30,'interpreter','latex');
saveas(figref,'figref','epsc')

figreftraj = figure('Position', get(0, 'Screensize'));
hold on
xlim([-150 150])
ylim([0 1000])
plot(ref_traj(:,1),ref_traj(:,2),'k-','LineWidth',2,'DisplayName','optimal control')
hold off
set(gca,'FontSize',30)
set(gca,'TickLabelInterpreter','latex');
xlabel('$x$ [m]','fontsize',30,'interpreter','latex')
ylabel('$y$ [m]','fontsize',30,'interpreter','latex')
% legend('Location','best','fontsize',30,'interpreter','latex')
title('Optimal Control Trajectory','fontsize',30,'interpreter','latex')
grid on
axis equal
saveas(figreftraj,'figreftraj','epsc')

