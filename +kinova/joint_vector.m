function q=joint_vector(q)
%JOINT_VECTOR Normalize a finite, real, six-joint numeric input.
assert(isnumeric(q)&&isreal(q)&&numel(q)==6&&all(isfinite(q(:))), ...
    'kinova:input','Expected six finite real joint values');q=double(q(:));
end
