function tau=inverse_dynamics(q,dq,ddq,m,Ftip)
%INVERSE_DYNAMICS Total joint-side effort required by the mechanical model.
% Ftip is the wrench the flange applies TO the environment, expressed in the
% flange frame; [moment;force]. Pass its negative for an applied environment wrench.
if nargin<5,Ftip=zeros(6,1);end
q=kinova.joint_vector(q);dq=kinova.joint_vector(dq);ddq=kinova.joint_vector(ddq);
Ftip=kinova.joint_vector(Ftip);
kin=rnea_shared_kinematics_MR(q,m.Mlist,m.Slist);
tau=InverseDynamicsSharedKinematics_MR(dq,ddq,m.gravity,Ftip,m.Glist,kin) ...
    +m.Japp*ddq+kinova.friction(dq,m);
end
