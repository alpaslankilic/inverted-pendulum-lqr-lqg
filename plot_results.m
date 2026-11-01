function plot_results(t,inputs,result,p,visible)
%PLOT_RESULTS Full test, applied forces, and estimated unmeasured velocities.
colors = [.4 .4 .4;0 .36 .60];
[fig,layout] = new_figure('Cart position and upright balance',2,visible);
ax = nexttile(layout);
hold(ax,'on');
plot(ax,t,inputs.reference,':','Color',[.6 .6 .6], ...
    'LineWidth',1.3,'DisplayName','Cart reference');
draw_comparison(ax,t,result,1,1,colors);
events(ax,p);
ylabel(ax,'Cart position [m]');
ylim(ax,[0,1.25*p.reference]);
title(ax,'(a) Position tracking on the nonlinear plant','FontWeight','normal');
legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
style(ax,t);
ax = nexttile(layout);
yline(ax,0,':','Color',[.6 .6 .6],'LineWidth',1.3, ...
    'DisplayName','Upright target (0 deg)');
draw_comparison(ax,t,result,3,180/pi,colors);
events(ax,p);
ylabel(ax,'Pendulum angle from upright [deg]');
xlabel(ax,'Time [s]');
title(ax,'(b) Balance through the disturbance','FontWeight','normal');
legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
style(ax,t);
save_figure(fig,'01_lqr_lqg_response',visible);

[fig,layout] = new_figure('Applied and commanded forces',2,visible);
ax = nexttile(layout);
stairs(ax,t,inputs.disturbance,'Color',[.45 .2 .55],'LineWidth',1.8);
events(ax,p);
ylabel(ax,'External cart force [N]');
ylim(ax,[-0.05,1.35*p.disturbanceForce]);
title(ax,'(a) Unmodeled force pulse at the plant input','FontWeight','normal');
style(ax,t);
ax = nexttile(layout);
hold(ax,'on');
plot(ax,t,result{1}.force,'--','Color',colors(1,:),'LineWidth',1.5, ...
    'DisplayName','Full-state LQR');
plot(ax,t,result{2}.force,'Color',colors(2,:),'LineWidth',1.5, ...
    'DisplayName','LQG: position + angle');
events(ax,p);
ylabel(ax,'Actuator force [N]');
xlabel(ax,'Time [s]');
title(ax,'(b) Same 8 N actuator limit for both controllers','FontWeight','normal');
legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
style(ax,t);
save_figure(fig,'02_force_response',visible);

[fig,layout] = new_figure('Kalman velocity estimates',2,visible);
columns = [2,4];
labels = {'Cart velocity [m/s]','Angular velocity [rad/s]'};
for k = 1:2
    ax = nexttile(layout);
    hold(ax,'on');
    plot(ax,t,result{2}.state(:,columns(k)),'Color',colors(2,:), ...
        'LineWidth',1.8,'DisplayName','True nonlinear state');
    plot(ax,t,result{2}.estimate(:,columns(k)),'--','Color',[.8 .32 .05], ...
        'LineWidth',1.1,'DisplayName','Kalman estimate');
    events(ax,p);
    ylabel(ax,labels{k});
    style(ax,t);
    if k==1
        title(ax,'(a) Velocity inferred from position and angle sensors', ...
            'FontWeight','normal');
        legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
    else
        title(ax,'(b) Angular velocity is not directly measured','FontWeight','normal');
        xlabel(ax,'Time [s]');
    end
end
save_figure(fig,'03_kalman_estimates',visible);

[fig,layout] = new_figure('Disturbance recovery detail',2,visible);
selected = t>=p.disturbanceTime-0.1;
for k = 1:2
    ax = nexttile(layout);
    hold(ax,'on');
    column = 1;
    scale = 1000;
    if k==2, column=3;scale=180/pi;end
    plot(ax,t(selected),scale*result{1}.state(selected,column),'--', ...
        'Color',colors(1,:),'LineWidth',1.5,'DisplayName','Full-state LQR');
    plot(ax,t(selected),scale*result{2}.state(selected,column), ...
        'Color',colors(2,:),'LineWidth',1.8,'DisplayName','LQG: position + angle');
    events(ax,p);
    style(ax,t(selected));
    if k==1
        ylabel(ax,'Cart position [mm]');
        title(ax,'(a) Position recovery after the pulse','FontWeight','normal');
        legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
    else
        yline(ax,0,':','Color',[.6 .6 .6],'LineWidth',1.3, ...
            'DisplayName','Upright target (0 deg)');
        ylabel(ax,'Angle from upright [deg]');
        xlabel(ax,'Time [s]');
        title(ax,'(b) Upright balance recovery','FontWeight','normal');
        legend(ax,'Location','southoutside','Orientation','horizontal','Box','off');
    end
end
save_figure(fig,'04_disturbance_detail',visible);
end

function draw_comparison(ax,t,result,column,scale,colors)
hold(ax,'on');
plot(ax,t,scale*result{1}.state(:,column),'--','Color',colors(1,:), ...
    'LineWidth',1.5,'DisplayName','Full-state LQR');
plot(ax,t,scale*result{2}.state(:,column),'Color',colors(2,:), ...
    'LineWidth',1.8,'DisplayName','LQG: position + angle');
end

function events(ax,p)
if ax.XLim(1)<p.referenceTime
    xline(ax,p.referenceTime,':','Color',[.55 .55 .55],'HandleVisibility','off');
end
xline(ax,p.disturbanceTime,'--',sprintf('Force pulse: %.3f s',p.disturbanceTime),'Color',[.45 .2 .55], ...
    'LabelOrientation','horizontal','FontSize',10,'HandleVisibility','off');
xline(ax,p.disturbanceTime+p.disturbanceDuration,':', ...
    'Color',[.45 .2 .55],'HandleVisibility','off');
end

function [fig,layout] = new_figure(name,panels,visible)
state = 'off';
if visible, state='on';end
fig = figure('Name',name,'Color','w','Visible',state, ...
    'Position',[100,80,980,450+140*(panels-1)]);
layout = tiledlayout(fig,panels,1,'TileSpacing','compact','Padding','loose');
end

function style(ax,t)
set(ax,'FontName','Arial','FontSize',12,'Box','off','TickDir','out', ...
    'YGrid','on','GridAlpha',.12,'LineWidth',.8);
xlim(ax,[t(1),t(end)]);
end

function save_figure(fig,name,visible)
exportgraphics(fig,fullfile('results',[name '.png']),'Resolution',200, ...
    'Padding',8,'Units','points');
exportgraphics(fig,fullfile('results',[name '.pdf']),'ContentType','vector', ...
    'Padding',8,'Units','points');
if ~visible, close(fig);end
end
