function [M,g,V]=lagrange_reference(q,m)
%LAGRANGE_REFERENCE Independent COM Jacobians; no spatial RNEA kernels.
% Intended for validation, not as the optimized simulation backend.
q=kinova.joint_vector(q);chain=kinova.frames(q,m);
M=m.Japp;g=zeros(6,1);V=0;
for i=1:6
    R=chain(1:3,1:3,i);r=R*m.com(:,i)+chain(1:3,4,i);
    Jv=zeros(3,6);Jw=zeros(3,6);
    for j=1:i
        w=chain(1:3,1:3,j)*m.axis_local(:,j);origin=chain(1:3,4,j);
        Jv(:,j)=cross(w,r-origin);Jw(:,j)=w;
    end
    M=M+m.mass(i)*Jv'*Jv+Jw'*R*m.Icom(:,:,i)*R'*Jw;
    g=g-m.mass(i)*Jv'*m.gravity;V=V-m.mass(i)*m.gravity'*r;
end
end
