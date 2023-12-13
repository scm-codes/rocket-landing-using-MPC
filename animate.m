%% 
figure;
ax = axes('XLim',[-100 100],'YLim',[-100 100],'ZLim',[0 1000]);
set(gca,"YTick",[])
% view([0 30]); 
view(3)
grid on; 
% axis tight;
daspect([1 1 1])
hold on
plot3(ref_traj(:,1), 0*ref_traj(:,1), ref_traj(:,2), 'k--')
xlabel('X[in m]','FontSize',20,'FontWeight','bold', 'Interpreter','latex'); 
zlabel('Y[in m]','FontSize',20,'FontWeight','bold', 'Interpreter','latex');
title("Case 3",'FontSize',20,'FontWeight','bold', 'Interpreter','latex')
set(gca,'TickLabelInterpreter','latex');


% create the surface
len_rocket = 75;
wid_rocket = 10;
len_flame = 25;
wid_flame = 5;
[x, y, z] = cylinder([1 1], 50);
h_rocket = surface(wid_rocket*x,wid_rocket*y,len_rocket*z, 'FaceColor','#808080','EdgeColor','texturemap');
h_flame = surface(wid_flame*x,wid_flame*y,len_flame*z, 'FaceColor','#ffa500','EdgeColor','texturemap');

% create hgtransform object
t_rocket = hgtransform('Parent',ax);
t_flame = hgtransform('Parent',ax);
set(h_rocket,'Parent',t_rocket)
set(h_flame, 'Parent',t_flame)
set(gcf,'Renderer','opengl')
drawnow
% Rz = eye(4);
% Sxy = Rz;

for ii = 1:length(xHistory)
    % Z-axis rotation matrix 
    
    Rt = makehgtform('translate',[xHistory(ii,1),0,xHistory(ii,2)]);
    Ry = makehgtform('yrotate',xHistory(ii,3));

    Rt_flame = makehgtform('translate',[0,0,-len_flame/2+10]);
    Ry_flame = makehgtform('yrotate', pi+uHistory(ii,2));
    Rscale_flame = makehgtform('scale',uHistory(ii,1)+1e-5);

    % set the hgtransform Matrix property
    set(t_flame,'Matrix',Rt*Ry*Rt_flame*Ry_flame*Rscale_flame)
    set(t_rocket,'Matrix',Rt*Ry)
    drawnow
    
    % frames
    frames(ii) = getframe(gcf);
    pause(0.01);
end

% save video
video = VideoWriter(['landing_case_', num2str(case_num)],'MPEG-4');
video.FrameRate = 10;
open(video)
writeVideo(video,frames);
close(video)

