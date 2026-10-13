function [H,K,V]=energy(q,dq,m)
%ENERGY Mechanical Hamiltonian with diagonal apparent inertia included once.
q=kinova.joint_vector(q);dq=kinova.joint_vector(dq);
[M,~]=kinova.dynamics(q,dq,m);K=.5*dq'*M*dq;
chain=kinova.frames(q,m);V=0;
for i=1:6
    r=chain(1:3,1:3,i)*m.com(:,i)+chain(1:3,4,i);
    V=V-m.mass(i)*m.gravity'*r;
end
H=K+V;
end
