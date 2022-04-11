function plotTrajData(time,xHistory,uHistory,refTraj,figLabels)
    

fig = figure('Position', get(0, 'Screensize'));
figtile = tiledlayout(4,2,'TileSpacing','tight','Padding','tight');

% time = 0:Ts:Duration;
% conversion 
d2r = pi/180;
r2d = 1/d2r;


nexttile
hold on
plot(time,xHistory(:,1),'r-','LineWidth',2,'DisplayName','MPC')
plot(refTraj(:,7),refTraj(:,1),'k--','LineWidth',2,'DisplayName','Reference')
xlim([0 16])
hold off
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$x$ [m]','fontsize',20,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,4),'r-','LineWidth',2,'DisplayName','MPC')
xlim([0 16])
hold off
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{x}$ [m/s]','fontsize',20,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,2),'r-','LineWidth',2,'DisplayName','MPC')
plot(refTraj(:,7),refTraj(:,2),'k--','LineWidth',2,'DisplayName','Reference')
yline(0,'b--','LineWidth',2,'DisplayName','constraint')
yline(1000,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$y$ [m]','fontsize',20,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,5),'r-','LineWidth',2,'DisplayName','MPC')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{y}$ [m/s]','fontsize',20,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,3)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
plot(refTraj(:,7),refTraj(:,3)*r2d,'k--','LineWidth',2,'DisplayName','Reference')
yline(90,'b--','LineWidth',2,'DisplayName','constraint')
yline(-90,'b--','LineWidth',2,'HandleVisibility','off')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\theta$ [deg]','fontsize',20,'interpreter','latex')
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

nexttile
hold on
plot(time,xHistory(:,6)*r2d,'r-','LineWidth',2,'DisplayName','MPC')
hold off
xlim([0 16])
set(gca,'FontSize',20)
set(gca,'TickLabelInterpreter','latex');
xlabel('$t$ [sec]','fontsize',20,'interpreter','latex')
ylabel('$\dot{\theta}$ [deg/s]','fontsize',20,'interpreter','latex')
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
legend('Location','northeastoutside','fontsize',20,'interpreter','latex')
grid on

sgtitle(figLabels{1},'fontsize',30,'interpreter','latex');
saveas(fig,figLabels{2},'epsc')

trajfig = figure('Position', get(0, 'Screensize'));
hold on
xlim([-150 150])
ylim([0 1000])
plot(xHistory(:,1),xHistory(:,2),'r-','LineWidth',2,'DisplayName','MPC')
plot(refTraj(:,1),refTraj(:,2),'k--','LineWidth',2,'DisplayName','Reference')
hold off
set(gca,'FontSize',30)
set(gca,'TickLabelInterpreter','latex');
xlabel('$x$ [m]','fontsize',30,'interpreter','latex')
ylabel('$y$ [m]','fontsize',30,'interpreter','latex')
legend('Location','best','fontsize',30,'interpreter','latex')
title(figLabels{3},'fontsize',30,'interpreter','latex')
grid on
axis equal
saveas(trajfig,figLabels{4},'epsc')

end