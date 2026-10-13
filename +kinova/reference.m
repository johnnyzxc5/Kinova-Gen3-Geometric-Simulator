function [q,dq,ddq]=reference(t,c,phase)
%REFERENCE Quintic joint-flat-output path using the Flatness-MPC phase evaluator.
if nargin<3,phase=ftv5.phase_plan(0,0,0,c.motion_time);end
[s,v,a]=ftv5.phase(phase,t);delta=c.qgoal(:)-c.q0(:);
q=c.q0(:)+delta*s;dq=delta*v;ddq=delta*a;
end
