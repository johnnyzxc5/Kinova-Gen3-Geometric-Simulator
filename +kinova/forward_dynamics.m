function ddq=forward_dynamics(q,dq,tau,m,external_wrench)
%FORWARD_DYNAMICS M*qdd = tau + Jbody'*F_environment - c - g - friction.
% external_wrench is applied BY the environment in the flange frame.
if nargin<5,external_wrench=zeros(6,1);end
tau=kinova.joint_vector(tau);external_wrench=kinova.joint_vector(external_wrench);
[M,b]=kinova.dynamics(q,dq,m);
if any(external_wrench),tau=tau+kinova.jacobian(q,m,'body')'*external_wrench;end
ddq=M\(tau-b);
end
