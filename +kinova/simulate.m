function log=simulate(m,c)
%SIMULATE Sampled computed effort, zero-order hold and mechanical RK4 plant.
% Reference generation is outside the plant. Controller gains are study settings.
% Limits are monitored at recorded integration stages; no collision model or
% invariance guarantee is implied. Continuous URDF joints remain unbounded.
if nargin<2,c=kinova.config();end
assert(c.control_dt>0&&c.substeps>=1&&c.substeps==floor(c.substeps));
assert(c.motion_time>0&&c.end_time>=c.motion_time);
assert(abs(c.end_time/c.control_dt-round(c.end_time/c.control_dt))<1e-9);
q0=kinova.joint_vector(c.q0);goal=kinova.joint_vector(c.qgoal);
assert(all(q0>=m.qmin&q0<=m.qmax)&&all(goal>=m.qmin&goal<=m.qmax));
controller=m;if ~isempty(c.controller_model),controller=c.controller_model;end
phase=ftv5.phase_plan(0,0,0,c.motion_time);
clock=tic;t=0:c.control_dt:c.end_time;N=numel(t);z=zeros(15,1);z(1:6)=q0;
log=struct('t',t,'q',zeros(6,N),'dq',zeros(6,N),'qref',zeros(6,N), ...
    'dqref',zeros(6,N),'tau_command',zeros(6,N),'tau_applied',zeros(6,N), ...
    'effectiveness',ones(6,N),'saturated',false(6,N),'H',zeros(1,N), ...
    'work_joint',zeros(1,N),'friction_loss',zeros(1,N),'work_environment',zeros(1,N), ...
    'position',zeros(3,N),'position_ref',zeros(3,N));
h=c.control_dt/c.substeps;limit_violation=0;velocity_utilization=0;
stage_min_margin=Inf;
for k=1:N
    q=z(1:6);dq=z(7:12);[qr,dqr,ddqr]=kinova.reference(t(k),c,phase);
    if isempty(c.control_function)
        virtual=ddqr+c.kp(:).*(qr-q)+c.kd(:).*(dqr-dq);
        command_raw=kinova.inverse_dynamics(q,dq,virtual,controller);
    else
        command_raw=kinova.joint_vector(c.control_function(t(k),q,dq,qr,dqr,ddqr,controller));
    end
    command=command_raw;
    if c.command_saturation,command=max(-m.taumax,min(m.taumax,command));end
    [alpha,~]=inputs(t(k),q,dq,c);actual=alpha.*command;
    log.q(:,k)=q;log.dq(:,k)=dq;log.qref(:,k)=qr;log.dqref(:,k)=dqr;
    log.tau_command(:,k)=command;log.tau_applied(:,k)=actual;
    log.effectiveness(:,k)=alpha;log.saturated(:,k)=abs(command_raw)>m.taumax+1e-12;
    log.H(k)=kinova.energy(q,dq,m);log.work_joint(k)=z(13);
    log.friction_loss(k)=z(14);log.work_environment(k)=z(15);
    T=kinova.fk(q,m);Tr=kinova.fk(qr,m);
    log.position(:,k)=T(1:3,4);log.position_ref(:,k)=Tr(1:3,4);
    if k<N
        for j=1:c.substeps
            tj=t(k)+(j-1)*h;
            k1=rhs(tj,z,command,m,c);
            k2=rhs(tj+h/2,z+h*k1/2,command,m,c);
            k3=rhs(tj+h/2,z+h*k2/2,command,m,c);
            k4=rhs(tj+h,z+h*k3,command,m,c);
            if c.capture_substeps
                stages=[z,z+h*k1/2,z+h*k2/2,z+h*k3,z+h*(k1+2*k2+2*k3+k4)/6];
                margins=[stages(1:6,:)-m.qmin;m.qmax-stages(1:6,:)];
                stage_min_margin=min(stage_min_margin,min(margins,[],'all'));
                limit_violation=max(limit_violation,max(-margins,[],'all'));
                velocity_utilization=max(velocity_utilization,max(abs(stages(7:12,:))./m.dqmax,[],'all'));
            end
            z=z+h*(k1+2*k2+2*k3+k4)/6;
            assert(all(isfinite(z)),'kinova:integration','Nonfinite state: reduce the step or check model/control');
        end
    end
end
error=log.q-log.qref;
log.energy_residual=log.H-log.H(1)-log.work_joint-log.work_environment+log.friction_loss;
log.metrics=struct('actuator_profile',m.actuator_profile,'friction_profile',m.friction_profile, ...
    'control_dt_s',c.control_dt,'plant_step_s',h,'duration_s',c.end_time, ...
    'max_joint_error_deg',max(abs(error),[],'all')*180/pi, ...
    'rms_joint_error_deg',sqrt(mean(error.^2,'all'))*180/pi, ...
    'max_position_error_mm',max(vecnorm(log.position-log.position_ref))*1000, ...
    'energy_balance_max_error_J',max(abs(log.energy_residual)), ...
    'minimum_recorded_limit_margin_rad',min([log.q-m.qmin;m.qmax-log.q],[],'all'), ...
    'minimum_stage_limit_margin_rad',stage_min_margin,'maximum_stage_limit_violation_rad',limit_violation, ...
    'max_stage_velocity_utilization',velocity_utilization, ...
    'max_command_utilization',max(abs(log.tau_command)./m.taumax,[],'all'), ...
    'saturated_joint_samples',nnz(log.saturated),'elapsed_seconds',toc(clock));
log.model_options=m.options;log.urdf_revision=m.urdf_revision;
end

function dz=rhs(t,z,command,m,c)
q=z(1:6);dq=z(7:12);[alpha,F]=inputs(t,q,dq,c);
[M,b]=kinova.dynamics(q,dq,m);actual=alpha.*command;environment=zeros(6,1);
if any(F),environment=kinova.jacobian(q,m,'body')'*F;end
ddq=M\(actual+environment-b);f=kinova.friction(dq,m);
dz=[dq;ddq;actual'*dq;f'*dq;environment'*dq];
end

function [alpha,F]=inputs(t,q,dq,c)
alpha=ones(6,1);F=zeros(6,1);
if ~isempty(c.effectiveness_function),alpha=kinova.joint_vector(c.effectiveness_function(t));end
if ~isempty(c.external_wrench_function),F=kinova.joint_vector(c.external_wrench_function(t,q,dq));end
assert(all(alpha>=0&alpha<=1),'kinova:input','Effectiveness must be in [0,1]');
end
