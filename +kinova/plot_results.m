function plot_results(log,m,out)
%PLOT_RESULTS Scientific trajectory/effort/energy plots, plus official mesh preview.
if ~isfolder(out),mkdir(out);end
fig=figure('Visible','off','Color','w','Position',[100,100,1250,800]);
close=onCleanup(@()delete(fig));tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
ax=nexttile;plot(ax,log.t,log.q*180/pi,'LineWidth',1.25);hold(ax,'on');
ax.ColorOrderIndex=1;plot(ax,log.t,log.qref*180/pi,'--','LineWidth',.7);grid(ax,'on');
title(ax,'Joint motion');xlabel(ax,'Time (s)');ylabel(ax,'Angle (deg)');
legend(ax,{'q_1','q_2','q_3','q_4','q_5','q_6'},'Location','eastoutside');
ax=nexttile;plot(ax,log.t,(log.q-log.qref)*180/pi,'LineWidth',1.2);grid(ax,'on');
title(ax,'Tracking error');xlabel(ax,'Time (s)');ylabel(ax,'Error (deg)');
ax=nexttile;plot(ax,log.t,log.tau_applied,'LineWidth',1.2);grid(ax,'on');
title(ax,'Applied joint effort');xlabel(ax,'Time (s)');ylabel(ax,'Torque (N m)');
ax=nexttile;plot(ax,log.t,log.energy_residual,'Color',[.07,.43,.54],'LineWidth',1.4);grid(ax,'on');
title(ax,'Mechanical energy balance residual');xlabel(ax,'Time (s)');ylabel(ax,'Residual (J)');
sgtitle(fig,'Kinova Gen3 | Geometric mechanical simulation','FontWeight','bold');
exportgraphics(fig,fullfile(out,'simulation_diagnostics.png'),'Resolution',160);
fig2=figure('Visible','off','Color','w','Position',[100,100,1000,760]);
close2=onCleanup(@()delete(fig2));ax=axes(fig2);kinova.draw_robot(log.q(:,end),m,ax);
hold(ax,'on');plot3(ax,log.position_ref(1,:),log.position_ref(2,:),log.position_ref(3,:), ...
    '--','Color',[.05,.48,.58],'LineWidth',1.7);
plot3(ax,log.position(1,:),log.position(2,:),log.position(3,:),'-','Color',[.95,.46,.17],'LineWidth',1.2);
title(ax,{'KINOVA GEN3 | SIX JOINTS','Official visual geometry + product-of-exponentials motion'},'FontWeight','bold');
exportgraphics(fig2,fullfile(out,'kinova_preview.png'),'Resolution',160);
end
