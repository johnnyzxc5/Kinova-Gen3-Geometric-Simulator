function [M,b,parts]=dynamics(q,dq,m)
%DYNAMICS Optimized M(q) and c(q,dq)+g(q)+f(dq), with shared Lie kinematics.
% Hoisted model constants, exact state cache, zero-state mass recursion, direct ad.
q=kinova.joint_vector(q);dq=kinova.joint_vector(dq);
kin=rnea_shared_kinematics_MR(q,m.Mlist,m.Slist);
M=MassMatrixZeroStateSharedKinematics_MR(m.Glist,kin)+m.Japp;
rigid=InverseDynamicsSharedKinematics_MR(dq,zeros(6,1),m.gravity,zeros(6,1),m.Glist,kin);
f=kinova.friction(dq,m);b=rigid+f;
if nargout>2
    g=InverseDynamicsSharedKinematics_MR(zeros(6,1),zeros(6,1),m.gravity,zeros(6,1),m.Glist,kin);
    parts=struct('gravity',g,'coriolis',rigid-g,'friction',f,'apparent_inertia',m.Japp);
end
end
