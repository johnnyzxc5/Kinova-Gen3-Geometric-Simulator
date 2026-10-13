function handles=draw_robot(q,m,ax,handles)
%DRAW_ROBOT Official Kinova visual meshes transformed by geometric kinematics.
% Returns reusable handles. Call again with handles to animate without reparsing.
if nargin<3||isempty(ax),ax=axes;end
chain=kinova.frames(q,m);root=fileparts(fileparts(mfilename('fullpath')));
names={'base_link','shoulder_link','bicep_link','forearm_link', ...
    'spherical_wrist_1_link','spherical_wrist_2_link','bracelet_no_vision_link'};
if nargin<4||isempty(handles)
    hold(ax,'on');handles=struct('patch',gobjects(1,7),'vertices',{cell(1,7)},'mesh',false);
    meshdir=fullfile(root,'sources','meshes');
    handles.mesh=all(cellfun(@(x)isfile(fullfile(meshdir,[x,'.STL'])),names));
    if handles.mesh
        for i=1:7
            [f,v]=kinova.read_stl(fullfile(meshdir,[names{i},'.STL']));handles.vertices{i}=v;
            color=[.84,.86,.89];if mod(i,2)==0,color=[.24,.27,.31];end
            handles.patch(i)=patch(ax,'Faces',f,'Vertices',v,'FaceColor',color, ...
                'EdgeColor','none','FaceLighting','gouraud','AmbientStrength',.35);
        end
        camlight(ax,'headlight');lighting(ax,'gouraud');
    end
    handles.line=plot3(ax,NaN,NaN,NaN,'-o','Color',[.05,.48,.58],'LineWidth',2,'MarkerSize',4);
    handles.flange=plot3(ax,NaN,NaN,NaN,'o','MarkerFaceColor',[.95,.46,.17],'MarkerEdgeColor','none','MarkerSize',7);
    grid(ax,'on');axis(ax,'equal');view(ax,140,22);
    xlabel(ax,'x (m)');ylabel(ax,'y (m)');zlabel(ax,'z (m)');
    xlim(ax,[-.85,.85]);ylim(ax,[-.85,.85]);zlim(ax,[-.1,1.25]);
end
if handles.mesh
    for i=1:7
        T=eye(4);if i>1,T=chain(:,:,i-1);end
        v=handles.vertices{i};handles.patch(i).Vertices=v*T(1:3,1:3)'+T(1:3,4)';
    end
    handles.line.Visible='off';
else
    pts=[zeros(3,1),reshape(chain(1:3,4,:),3,[])];
    set(handles.line,'XData',pts(1,:),'YData',pts(2,:),'ZData',pts(3,:));
end
set(handles.flange,'XData',chain(1,4,7),'YData',chain(2,4,7),'ZData',chain(3,4,7));
end
