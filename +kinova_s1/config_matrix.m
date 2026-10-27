function [X, max_norm_err] = config_matrix(X_in, tol)
%CONFIG_MATRIX Validate and normalize Kinova (S1)^6 configuration matrix.
%
% Formats accepted:
%   - 2-by-6 matrix where column i is [c_i; s_i]
%   - 12-by-1 or 1-by-12 vector (reshaped column-wise into 2-by-6)
%   - 6-by-2 matrix (transposed into 2-by-6)
%
% Outputs:
%   - X: 2-by-6 double matrix
%   - max_norm_err: max_i |c_i^2 + s_i^2 - 1|

if nargin < 2
    tol = [];
end

assert(isnumeric(X_in) && all(isfinite(X_in(:))), ...
    'kinova_s1:config_matrix', 'Configuration X must be finite numeric');

sz = size(X_in);
if isequal(sz, [2, 6])
    X = double(X_in);
elseif isequal(sz, [12, 1]) || isequal(sz, [1, 12])
    X = reshape(double(X_in), 2, 6);
elseif isequal(sz, [6, 2])
    X = double(X_in)';
else
    error('kinova_s1:config_matrix', ...
        'Configuration must be 2-by-6, 6-by-2, or 12-element vector (got %dx%d)', sz(1), sz(2));
end

col_sq = sum(X.^2, 1);
max_norm_err = max(abs(col_sq - 1));

if ~isempty(tol)
    if any(col_sq < eps^2)
        error('kinova_s1:config_matrix_degenerate', ...
            'Columns of X must be non-zero (degenerate column detected with norm < eps)');
    end
    if max_norm_err > tol
        error('kinova_s1:unit_norm', ...
            'Columns of X must have unit norm on S^1 (max error: %e > tol: %e)', ...
            max_norm_err, tol);
    end
end
end
