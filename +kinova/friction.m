function f=friction(dq,m)
%FRICTION Passive smooth Coulomb + viscous effort; physical plant law.
% Coefficients from Muraccioli et al. RSS2025 eq.12, transferred by actuator type.
% The acceleration-dependent start-up compensation is deliberately separate
% from the physical law. This smooth law does not implement set-valued stiction.
dq=kinova.joint_vector(dq);x=m.friction.slope*dq;
f=m.friction.viscous.*dq+m.friction.coulomb.*tanh(x);
switch char(m.friction_profile)
    case 'muraccioli_smooth'
    case 'shape_blend',f=tanh(abs(x)).*f;
    case 'none',f=zeros(6,1);
end
end
