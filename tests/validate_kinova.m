function report=validate_kinova()
%VALIDATE_KINOVA Independent mechanics + Lie identities + optimized-kernel parity.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);setup_kinova();
rng(13102026,'twister');m=kinova.model();z=zeros(6,1);ep=1e-5;
report=struct('seed',13102026,'states',40,'matlab_version',version, ...
    'mass_reference_error',0,'gravity_reference_error_Nm',0,'coriolis_reference_error_Nm',0, ...
    'fk_chain_error',0,'jacobian_finite_difference_error',0,'mass_asymmetry',0, ...
    'min_mass_eigenvalue',Inf,'potential_gradient_error_Nm',0,'power_identity_error',0, ...
    'stock_inverse_dynamics_error_Nm',0,'stock_mass_error',0, ...
    'forward_inverse_roundtrip_error',0,'wrench_duality_error',0, ...
    'minimum_friction_power_W',Inf,'adjoint_exponential_identity_error',0);
for k=1:report.states
    q=[(2*rand-1)*pi;(2*rand-1)*1.9;(2*rand-1)*2.1;(2*rand-1)*pi;(2*rand-1)*1.8;(2*rand-1)*pi];
    dq=randn(6,1);ddq=randn(6,1);F=randn(6,1);
    [M,~,parts]=kinova.dynamics(q,dq,m);[ML,gL]=kinova.lagrange_reference(q,m);
    [~,Tchain]=kinova.frames(q,m);T=kinova.fk(q,m);
    report.fk_chain_error=max(report.fk_chain_error,norm(T-Tchain,'fro'));
    report.mass_reference_error=max(report.mass_reference_error,norm(M-ML,'fro')/norm(ML,'fro'));
    report.gravity_reference_error_Nm=max(report.gravity_reference_error_Nm,norm(parts.gravity-gL));
    report.mass_asymmetry=max(report.mass_asymmetry,norm(M-M','fro'));
    report.min_mass_eigenvalue=min(report.min_mass_eigenvalue,min(eig((M+M')/2)));
    Js=kinova.jacobian(q,m,'space');Jb=kinova.jacobian(q,m,'body');
    derivative=zeros(6,6,6);gradient=zeros(6,1);Jfd=zeros(6,6);
    for j=1:6
        d=z;d(j)=ep;[Mp,~,Vp]=kinova.lagrange_reference(q+d,m);[Mm,~,Vm]=kinova.lagrange_reference(q-d,m);
        derivative(:,:,j)=(Mp-Mm)/(2*ep);gradient(j)=(Vp-Vm)/(2*ep);
        Tp=kinova.fk(q+d,m);Tm=kinova.fk(q-d,m);twist=(Tp-Tm)/(2*ep)*TransInv(T);
        Jfd(:,j)=[so3ToVec(twist(1:3,1:3));twist(1:3,4)];
    end
    Mdot=zeros(6);gradK=z;
    for j=1:6,Mdot=Mdot+derivative(:,:,j)*dq(j);gradK(j)=dq'*derivative(:,:,j)*dq;end
    cL=Mdot*dq-.5*gradK;
    report.coriolis_reference_error_Nm=max(report.coriolis_reference_error_Nm,norm(parts.coriolis-cL));
    report.potential_gradient_error_Nm=max(report.potential_gradient_error_Nm,norm(parts.gravity-gradient));
    report.power_identity_error=max(report.power_identity_error,abs(dq'*parts.coriolis-.5*dq'*Mdot*dq));
    report.jacobian_finite_difference_error=max(report.jacobian_finite_difference_error,norm(Js-Jfd,'fro'));
    tr=InverseDynamics(q,dq,ddq,m.gravity,F,m.Mlist,m.Glist,m.Slist)+m.Japp*ddq+kinova.friction(dq,m);
    to=kinova.inverse_dynamics(q,dq,ddq,m,F);
    report.stock_inverse_dynamics_error_Nm=max(report.stock_inverse_dynamics_error_Nm,norm(tr-to));
    Mr=MassMatrix(q,m.Mlist,m.Glist,m.Slist)+m.Japp;
    report.stock_mass_error=max(report.stock_mass_error,norm(Mr-M,'fro'));
    recovered=kinova.forward_dynamics(q,dq,to,m,-F);
    report.forward_inverse_roundtrip_error=max(report.forward_inverse_roundtrip_error,norm(recovered-ddq));
    report.wrench_duality_error=max(report.wrench_duality_error,abs(F'*(Jb*dq)-(Jb'*F)'*dq));
    report.minimum_friction_power_W=min(report.minimum_friction_power_W,dq'*parts.friction);
    X=randn(6,1)*.2;
    report.adjoint_exponential_identity_error=max(report.adjoint_exponential_identity_error, ...
        norm(T*MatrixExp6(VecTose3(X))*TransInv(T)-MatrixExp6(VecTose3(Adjoint(T)*X)),'fro'));
end
expected=[.0930079;.0930079;.0930079;.0560033;.0560033;.0560033];
assert(norm(diag(m.Japp)-expected)<1e-12,'kinova:test','Apparent inertia conversion');
assert(all(isinf(m.qmax([1,4,6])))&&all(isinf(m.qmin([1,4,6]))));
% Cache invalidation when only model or joint state changes.
other=kinova.model(struct('actuator_profile','rigid_links','friction_profile','none'));
altered=m;altered.Mlist(1,4,3)=altered.Mlist(1,4,3)+.01;
q=z+.15;rnea_shared_kinematics_MR(q,m.Mlist,m.Slist,'reset');
kinova.dynamics(q,q,m);[Mc,~]=kinova.dynamics(q,q,altered);
Mgold=MassMatrix(q,altered.Mlist,altered.Glist,altered.Slist)+altered.Japp;
assert(norm(Mc-Mgold,'fro')<1e-10,'kinova:test','Cache invalidation');
for profile={'muraccioli_smooth','shape_blend','none'}
    fm=kinova.model(struct('friction_profile',profile{1}));
    for vel=[-1,-.01,0,.01,1],v=vel*ones(6,1);assert(v'*kinova.friction(v,fm)>=-1e-12);end
end
assert(all(kinova.friction(z,m)==0));
[Mo,~]=kinova.dynamics(q,z,other);
assert(norm(Mo-MassMatrix(q,other.Mlist,other.Glist,other.Slist),'fro')<1e-10);
report.pass=report.mass_reference_error<1e-10&&report.gravity_reference_error_Nm<1e-10 ...
    &&report.coriolis_reference_error_Nm<1e-6&&report.fk_chain_error<1e-10 ...
    &&report.jacobian_finite_difference_error<1e-7&&report.mass_asymmetry<1e-10 ...
    &&report.min_mass_eigenvalue>0&&report.potential_gradient_error_Nm<1e-6 ...
    &&report.power_identity_error<1e-6&&report.stock_inverse_dynamics_error_Nm<1e-10 ...
    &&report.stock_mass_error<1e-10&&report.forward_inverse_roundtrip_error<1e-9 ...
    &&report.wrench_duality_error<1e-12&&report.minimum_friction_power_W>=0 ...
    &&report.adjoint_exponential_identity_error<1e-10;
kinova.write_json(fullfile(root,'results','validation.json'),report);
disp(jsonencode(report,PrettyPrint=true));assert(report.pass,'kinova:test','Model validation failed');
end
