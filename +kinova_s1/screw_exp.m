function T = screw_exp(S, x, u_mid, q_mid)
%SCREW_EXP Algebraic SE(3) exponential map exp([S]*q) evaluated directly on S^1.
%
% Implements Theorem 3.1:
% For a unit-axis revolute screw S = [w; v] with ||w|| = 1 and zero pitch (w'*v = 0),
% the SE(3) exponential depends purely algebraically on x = [c; s] in S^1:
%
%   exp([S], x) = [ I + s*[w]_x + (1-c)*[w]_x^2,   s*v + (1-c)*([w]_x * v);
%                   0, 0, 0,                       1 ]
%
% If S has non-zero helical pitch (w'*v ~= 0, e.g., in perturbed screw tests),
% the axial translation term (q - s)*(w'*v)*w uses the centered lift q(x).
%
% Inputs:
%   S: 6-by-1 screw axis [w; v]
%   x: 2-by-1 unit vector [c; s] on S^1
%   u_mid: (optional, default [1;0]) 2-by-1 arc center unit vector [cos(m); sin(m)]
%   q_mid: (optional, default 0) scalar arc midpoint [rad]
% Output:
%   T: 4-by-4 matrix in SE(3)

c = x(1);
s = x(2);
w = S(1:3);
v = S(4:6);

wnorm2 = w(1)*w(1) + w(2)*w(2) + w(3)*w(3);
if abs(wnorm2 - 1) < 1e-12
    % Skew-symmetric matrix Omega = [w]_x
    wx = w(1); wy = w(2); wz = w(3);
    Omega = [  0, -wz,  wy; ...
              wz,   0, -wx; ...
             -wy,  wx,   0 ];
    Omega2 = Omega * Omega;
    one_minus_c = 1 - c;

    R = eye(3) + s * Omega + one_minus_c * Omega2;
    p = s * v + one_minus_c * (Omega * v);

    % Check for non-zero helical pitch h_pitch = w' * v
    pitch = wx * v(1) + wy * v(2) + wz * v(3);
    if abs(pitch) > 1e-14
        if nargin >= 4 && ~isempty(u_mid) && ~isempty(q_mid)
            cm = u_mid(1); sm = u_mid(2);
            q = q_mid + atan2(s * cm - c * sm, c * cm + s * sm);
        else
            q = atan2(s, c);
        end
        p = p + (q - s) * pitch * w;
    end

    T = [R, p; 0, 0, 0, 1];
else
    % General fallback for non-unit angular norm (e.g. prismatic or scaled screw)
    if nargin >= 4 && ~isempty(u_mid) && ~isempty(q_mid)
        cm = u_mid(1); sm = u_mid(2);
        q = q_mid + atan2(s * cm - c * sm, c * cm + s * sm);
    else
        q = atan2(s, c);
    end
    T = MatrixExp6(VecTose3(S * q));
end
end
