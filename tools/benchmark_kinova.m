function report=benchmark_kinova()
%BENCHMARK_KINOVA Matched complete dynamics work; steady-state MATLAB timings.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);setup_kinova();
m=kinova.model();rng(13102026,'twister');Q=randn(6,100)*.5;DQ=randn(6,100)*.2;
for i=1:100,kinova.dynamics(Q(:,i),DQ(:,i),m);stock(Q(:,i),DQ(:,i),m);end
baseline=zeros(1,5);optimized=zeros(1,5);
for r=1:5
    tic;for i=1:100,stock(Q(:,i),DQ(:,i),m);end;baseline(r)=toc/100;
    tic;for i=1:100,kinova.dynamics(Q(:,i),DQ(:,i),m);end;optimized(r)=toc/100;
end
report=struct('matlab_version',version,'states',100,'repeats',5, ...
    'stock_median_us',median(baseline)*1e6,'optimized_median_us',median(optimized)*1e6, ...
    'speedup',median(baseline)/median(optimized), ...
    'scope','M and c+g+f, changing q and dq, warmed serial CPU. No rendering or integration.');
kinova.write_json(fullfile(root,'results','benchmark.json'),report);disp(report);
end
function [M,b]=stock(q,dq,m)
M=MassMatrix(q,m.Mlist,m.Glist,m.Slist)+m.Japp;
b=InverseDynamics(q,dq,zeros(6,1),m.gravity,zeros(6,1),m.Mlist,m.Glist,m.Slist)+kinova.friction(dq,m);
end
