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

