function [chain,T]=frames(q,m)
%FRAMES Product of relative poses and exp([joint twist]*q) in SE(3).
q=kinova.joint_vector(q);T=eye(4);chain=zeros(4,4,7);
for i=1:7
    T=T*m.Mlist(:,:,i);
    if i<=6,T=T*MatrixExp6(VecTose3([m.axis_local(:,i);zeros(3,1)]*q(i)));end
    chain(:,:,i)=T;
end
end
