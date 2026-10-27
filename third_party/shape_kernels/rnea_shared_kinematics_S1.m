function kin = rnea_shared_kinematics_S1(X, Mlist, Slist, mode)
%RNEA_SHARED_KINEMATICS_S1 Exact Lie kinematics on (S1)^n with Adjoint hoisting and S^1 caching.
%
% For pure-z revolute joints (all 6 nominal Kinova joints), exploits the Lie-group
% homomorphism Ad_{T_z(x_i)^T * M_i^{-1}} = diag(R_z(x_i)^T, R_z(x_i)^T) * Ad_{M_i^{-1}},
% eliminating MatrixExp6, VecTose3, Adjoint, TransToRp, and VecToso3 calls at runtime.
% Includes automatic centered-lift screw_exp fallback for general or perturbed screws.
%
% Inputs:
%   X: 2-by-n configuration matrix on (S1)^n
%   Mlist: 4-by-4-by-(n+1) relative home transforms
%   Slist: 6-by-n space screws
%   mode: (optional) 'cached' (default), 'reset', 'stats', 'legacy'

persistent cached_Mlist cached_Slist cached_n cached_Ai cached_invM cached_Ad_invM cached_AdTi_end
persistent cached_is_pure_z cached_qmid cached_umid cached_X_u64 cached_kin
persistent stats_calls stats_hits stats_misses stats_rebuilds

if isempty(stats_calls)
    stats_calls = uint64(0);
    stats_hits = uint64(0);
    stats_misses = uint64(0);
    stats_rebuilds = uint64(0);
end

if nargin >= 4 && strcmp(mode, 'reset')
    cached_Mlist = []; cached_Slist = []; cached_n = [];
    cached_Ai = []; cached_invM = []; cached_Ad_invM = []; cached_AdTi_end = [];
    cached_is_pure_z = []; cached_qmid = []; cached_umid = [];
    cached_X_u64 = []; cached_kin = [];
    stats_calls = uint64(0); stats_hits = uint64(0);
    stats_misses = uint64(0); stats_rebuilds = uint64(0);
    kin = struct('Ai', zeros(6, 0), 'AdTi', zeros(6, 6, 0), 'n', 0);
    return;
end

if nargin >= 4 && strcmp(mode, 'stats')
    kin = struct('calls', stats_calls, 'hits', stats_hits, ...
                 'misses', stats_misses, 'rebuilds', stats_rebuilds);
    return;
end

X = kinova_s1.config_matrix(X);
n = size(X, 2);

default_qmid = zeros(n, 1);
default_umid = [cos(default_qmid)'; sin(default_qmid)'];

if nargin >= 4 && strcmp(mode, 'legacy')
    Mi = eye(4);
    Ai = zeros(6, n);
    AdTi = zeros(6, 6, n + 1);
    AdTi(:, :, n + 1) = Adjoint(TransInv(Mlist(:, :, n + 1)));
    for i = 1:n
        Mi = Mi * Mlist(:, :, i);
        Ai(:, i) = Adjoint(TransInv(Mi)) * Slist(:, i);
        x_inv = [X(1, i); -X(2, i)];
        u_inv = [default_umid(1, i); -default_umid(2, i)];
        T_exp = kinova_s1.screw_exp(Ai(:, i), x_inv, u_inv, -default_qmid(i));
        AdTi(:, :, i) = Adjoint(T_exp * TransInv(Mlist(:, :, i)));
    end
    kin = struct('Ai', Ai, 'AdTi', AdTi, 'n', n);
    return;
end

stats_calls = stats_calls + 1;

% Invariant Hoisting: rebuild only if structural parameters changed
need_rebuild = isempty(cached_Mlist) || isempty(cached_Slist) || ...
    ~isequal(size(cached_Mlist), size(Mlist)) || ...
    ~isequal(size(cached_Slist), size(Slist)) || ...
    ~strcmp(class(cached_Mlist), class(Mlist)) || ...
    ~strcmp(class(cached_Slist), class(Slist)) || ...
    ~isequal(cached_Mlist, Mlist) || ...
    ~isequal(cached_Slist, Slist);

if need_rebuild
    cached_n = n;
    cached_Mlist = Mlist;
    cached_Slist = Slist;
    Mi = eye(4);
    cached_Ai = zeros(6, n);
    cached_invM = zeros(4, 4, n);
    cached_Ad_invM = zeros(6, 6, n);
    cached_is_pure_z = false(1, n);
    for i = 1:n
        Mi = Mi * Mlist(:, :, i);
        cached_Ai(:, i) = Adjoint(TransInv(Mi)) * Slist(:, i);
        cached_invM(:, :, i) = TransInv(Mlist(:, :, i));
        cached_Ad_invM(:, :, i) = Adjoint(cached_invM(:, :, i));
        cached_is_pure_z(i) = norm(cached_Ai(:, i) - [0; 0; 1; 0; 0; 0], inf) < 1e-12;
    end
    cached_AdTi_end = Adjoint(TransInv(Mlist(:, :, n + 1)));
    cached_qmid = default_qmid;
    cached_umid = default_umid;
    cached_X_u64 = [];
    cached_kin = [];
    stats_rebuilds = stats_rebuilds + 1;
end

% STATE CACHE LOOKUP: exact raw bitwise uint64 comparison across doubles
X_u64 = typecast(X(:), 'uint64');
if ~need_rebuild && ~isempty(cached_X_u64)
    if all(cached_X_u64 == X_u64)
        stats_hits = stats_hits + 1;
        kin = cached_kin;
        return;
    end
end

stats_misses = stats_misses + 1;

AdTi = zeros(6, 6, n + 1);
AdTi(:, :, n + 1) = cached_AdTi_end;

for i = 1:n
    ci = X(1, i);
    si = X(2, i);
    if cached_is_pure_z(i)
        % Exact Adjoint homomorphism: diag(R_z(x_i)^T, R_z(x_i)^T) * Ad_{M_i^{-1}}
        B = cached_Ad_invM(:, :, i);
        AdTi(:, :, i) = [ ci * B(1, :) + si * B(2, :); ...
                         -si * B(1, :) + ci * B(2, :); ...
                          B(3, :); ...
                          ci * B(4, :) + si * B(5, :); ...
                         -si * B(4, :) + ci * B(5, :); ...
                          B(6, :) ];
    else
        % General screw fallback (e.g. artificial geometry or screw perturbation in tests)
        x_inv = [ci; -si];
        u_inv = [cached_umid(1, i); -cached_umid(2, i)];
        T_exp = kinova_s1.screw_exp(cached_Ai(:, i), x_inv, u_inv, -cached_qmid(i));
        AdTi(:, :, i) = Adjoint(T_exp * cached_invM(:, :, i));
    end
end

kin = struct('Ai', cached_Ai, 'AdTi', AdTi, 'n', n);

cached_X_u64 = X_u64;
cached_kin = kin;
end
