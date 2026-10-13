function root=setup_kinova()
%SETUP_KINOVA Add only this repository's runtime paths. Base MATLAB suffices.
root=fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'third_party','modern_robotics'), ...
    fullfile(root,'third_party','shape_kernels'),fullfile(root,'third_party','flatness'));
end
