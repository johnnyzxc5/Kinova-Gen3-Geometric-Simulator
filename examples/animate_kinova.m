function animate_kinova(log,m)
%ANIMATE_KINOVA Interactive visual playback; dynamics have already been computed.
if nargin<2,m=kinova.model();end
fig=figure('Color','w','Name','Kinova Gen3 geometric simulation');ax=axes(fig);h=[];
for k=1:10:numel(log.t)
    if ~isgraphics(fig),break;end
    h=kinova.draw_robot(log.q(:,k),m,ax,h);title(ax,sprintf('Kinova Gen3 | t = %.2f s',log.t(k)));
    drawnow;pause(.01);
end
end
