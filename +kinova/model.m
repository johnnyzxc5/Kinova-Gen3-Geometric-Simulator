function m=model(options)
%MODEL Manufacturer rigid links + a configurable diagonal apparent inertia.
% Twists are [angular; linear]. Inertias use SI units and link-local frames.
% The default actuator constants are Cholet commercial proxies, not identified
% motor parameters of the physical Kinova. No electrical dynamics are modeled.
if nargin<1, options=struct(); end
assert(isstruct(options)&&isscalar(options),'kinova:model','Options must be a scalar struct');
defaults=struct('actuator_profile','cholet_apparent','custom_apparent_inertia',[], ...
    'friction_profile','muraccioli_smooth','friction_slope',50,'gravity',[0;0;-9.80665]);
names=fieldnames(options);
for i=1:numel(names)
    assert(isfield(defaults,names{i}),'kinova:model','Unknown model option: %s',names{i});
    defaults.(names{i})=options.(names{i});
end
o=defaults;
root=fileparts(fileparts(mfilename('fullpath')));
p=jsondecode(fileread(fullfile(root,'sources','parameters.json')));
m=struct('n',6,'name','Kinova Gen3 6-DoF, no vision, bare arm', ...
    'urdf_revision',p.urdf_revision,'options',o,'gravity',o.gravity(:), ...
    'actuator_profile',o.actuator_profile,'friction_profile',o.friction_profile);
assert(numel(m.gravity)==3&&all(isfinite(m.gravity)),'kinova:model','Invalid gravity');
m.Mlist=repmat(eye(4),1,1,7);m.Glist=zeros(6,6,6);m.Slist=zeros(6,6);
m.home_links=zeros(4,4,7);m.axis_local=zeros(3,6);
m.mass=zeros(6,1);m.com=zeros(3,6);m.Icom=zeros(3,3,6);
m.qmin=-inf(6,1);m.qmax=inf(6,1);m.taumax=zeros(6,1);m.dqmax=zeros(6,1);
T=eye(4);
for i=1:7
    if iscell(p.joints),j=p.joints{i};else,j=p.joints(i);end
    Ti=eye(4);Ti(1:3,1:3)=kinova.rpy_rotation(j.rpy);Ti(1:3,4)=j.xyz(:);
    m.Mlist(:,:,i)=Ti; % Relative transform, never the accumulated home pose.
    T=T*Ti;m.home_links(:,:,i)=T;
    if i<=6
        a=j.axis(:);assert(abs(norm(a)-1)<1e-12,'kinova:model','Joint axis must be unit length');
        m.axis_local(:,i)=a;w=T(1:3,1:3)*a;r=T(1:3,4);
        m.Slist(:,i)=[w;-cross(w,r)];m.mass(i)=j.mass;m.com(:,i)=j.com(:);
        v=j.inertia(:);I=[v(1),v(2),v(3);v(2),v(4),v(5);v(3),v(5),v(6)];
        R=kinova.rpy_rotation(j.inertia_rpy);I=R*I*R';m.Icom(:,:,i)=I;
        C=VecToso3(m.com(:,i));m.Glist(:,:,i)=[I-m.mass(i)*C*C,m.mass(i)*C;-m.mass(i)*C,m.mass(i)*eye(3)];
        m.taumax(i)=j.effort;m.dqmax(i)=j.velocity;
        if ~strcmp(j.type,'continuous'),m.qmin(i)=j.lower;m.qmax(i)=j.upper;end
    end
end
m.home=T;m.Blist=Adjoint(TransInv(T))*m.Slist;
m.actuator= p.actuator_proxy;
switch char(o.actuator_profile)
    case 'cholet_apparent'
        % Jr is already referred to the reducer output, so do not scale it again.
        apparent=p.actuator_proxy.motor_inertia(:).*p.actuator_proxy.reduction(:).^2 ...
            +p.actuator_proxy.reducer_output_inertia(:);
    case 'rigid_links'
        apparent=zeros(6,1);
    case 'custom'
        apparent=o.custom_apparent_inertia(:);
    otherwise
        error('kinova:model','Unknown actuator profile');
end
assert(numel(apparent)==6&&all(isfinite(apparent))&&all(apparent>=0), ...
    'kinova:model','Apparent inertia must contain six finite nonnegative values');
m.Japp=diag(apparent);
m.friction=struct('viscous',p.friction.viscous(:),'coulomb',p.friction.coulomb(:), ...
    'slope',o.friction_slope,'source',p.friction.source);
assert(isscalar(o.friction_slope)&&isfinite(o.friction_slope)&&o.friction_slope>0, ...
    'kinova:model','Friction slope must be positive');
assert(any(strcmp(char(o.friction_profile),{'muraccioli_smooth','shape_blend','none'})), ...
    'kinova:model','Unknown friction profile');
m.scope='Rigid bare arm, joint-side effort, approximate apparent inertia; no hardware identification claim';
end
