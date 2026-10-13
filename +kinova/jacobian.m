function J=jacobian(q,m,frame)
%JACOBIAN Space or flange-body twist Jacobian, rows [angular; linear].
if nargin<3,frame='space';end
q=kinova.joint_vector(q);J=JacobianSpace(m.Slist,q);
switch char(frame)
    case 'space'
    case 'body',J=Adjoint(TransInv(kinova.fk(q,m)))*J;
    otherwise,error('kinova:frame','Frame must be space or body');
end
end
