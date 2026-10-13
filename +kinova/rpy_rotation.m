function R=rpy_rotation(rpy)
%RPY_ROTATION URDF fixed-axis roll-pitch-yaw: Rz(yaw)*Ry(pitch)*Rx(roll).
a=rpy(:);assert(numel(a)==3&&all(isfinite(a)));
cr=cos(a(1));sr=sin(a(1));cp=cos(a(2));sp=sin(a(2));cy=cos(a(3));sy=sin(a(3));
R=[cy,-sy,0;sy,cy,0;0,0,1]*[cp,0,sp;0,1,0;-sp,0,cp]*[1,0,0;0,cr,-sr;0,sr,cr];
end
