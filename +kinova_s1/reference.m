function [Xref, wref, dwref, qref] = reference(t, c, m, phase, X0, d_box)
%REFERENCE Intrinsic (S1)^6 reference trajectory along the admissible configuration space.
%
% Given initial configuration X0 and displacement d_box, interpolates along the
% configuration manifold via the exponential map:
%   Xref(t) = Exp_{S1}( s(t) * d_box, X0 )
%   wref(t) = v(t) * d_box
%   dwref(t) = a(t) * d_box
%   qref(t) = q0 + d_box * s(t)
%
% For continuous joints, d_box can represent any continuous displacement
% (including multiple turns |d_box_i| > 2*pi). The group action Exp_{S1}
% maps smoothly and periodically onto (S1)^6 without singularities.
%
% Inputs:
%   t: current time [s]
%   c: configuration struct from kinova.config()
%   m: (optional) model struct from kinova_s1.model()
%   phase: (optional) phase plan from ftv5.phase_plan()
%   X0: (optional) 2-by-6 initial configuration on (S1)^6
%   d_box: (optional) 6-by-1 displacement vector [rad]
% Outputs:
%   Xref: 2-by-6 reference configuration on (S1)^6
%   wref: 6-by-1 reference angular velocity [rad/s]
%   dwref: 6-by-1 reference angular acceleration [rad/s^2]
%   qref: 6-by-1 reference joint angles in R^6 (with continuous winding)

if nargin < 3 || isempty(m)
    m = kinova_s1.model();
end
if nargin < 4 || isempty(phase)
    phase = ftv5.phase_plan(0, 0, 0, c.motion_time);
end
if nargin < 5 || isempty(X0)
    X0 = kinova_s1.from_q(c.q0);
end
if nargin < 6 || isempty(d_box)
    if isfield(c, 'qgoal') && isfield(c, 'q0')
        d_box = kinova.joint_vector(c.qgoal) - kinova.joint_vector(c.q0);
    else
        Xgoal = kinova_s1.from_q(c.qgoal);
        d_box = kinova_s1.box_diff(Xgoal, X0, m);
    end
end

[s, v, a] = ftv5.phase(phase, t);
dtheta = d_box * s;
Xref = kinova_s1.exp_s1(dtheta, X0, true);
wref = d_box * v;
dwref = d_box * a;

if nargout > 3
    qref = kinova.joint_vector(c.q0) + d_box * s;
end
end
