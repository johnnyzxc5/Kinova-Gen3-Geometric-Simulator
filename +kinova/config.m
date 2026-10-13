function c=config()
%CONFIG Reproducible joint-space study settings, separate from robot data.
c.control_dt=.004;c.substeps=8;c.motion_time=3.0;c.end_time=3.6;
c.q0=[0;.35;-.7;0;.35;0];c.qgoal=[.35;.6;-.95;.3;.55;-.25];
c.kp=64*ones(6,1);c.kd=16*ones(6,1);
c.command_saturation=true;
c.controller_model=[]; % Empty uses the same model as the plant.
c.control_function=[]; % @(t,q,dq,qr,dqr,ddqr,m_controller) returns joint effort.
c.external_wrench_function=[]; % @(t,q,dq) returns flange-body environment wrench.
c.effectiveness_function=[]; % @(t) returns six factors in [0,1].
c.capture_substeps=true;c.save_results=true;
end
