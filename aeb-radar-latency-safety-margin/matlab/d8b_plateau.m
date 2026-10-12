%% d8b_plateau: 联合预算精细扫描 + Figure 7  (100 km/h × Δt精网格 × σ) 约15分钟
% 评审 P1#3(为什么是 0.20 不是 0.15?) + P2#4(Figure 7 配图)。
% 预测: 碰撞率随 Δt 呈分段常数平台, 边界在 Δt≈0.14/0.24/0.34/0.44 s;
%       0.15/0.175/0.20 同属一个平台(≈4.3%), 膝盖(下缘)≈0.14 s;
%       Δt≤0.125 时仅 +0.18/+0.28 迟到 bin 有效(≈1-3%)。
clear; clc; close all; rng('default');
D0=60; Dtrig=40; Nmc=1000; v=100/3.6; S=v^2/16;
dList = [0.05 0.10 0.125 0.15 0.175 0.20 0.225 0.25 0.30 0.35 0.40];
sigList = [0 3];
OUT = 'C:\deepseek\论文\文件列表';

% ---- 100 km/h 场景 (同 d8) ----
scenario = drivingScenario('SampleTime',0.1,'StopTime',10);
road(scenario,[0 0;400 0],'Lanes',lanespec(2));
ego = vehicle(scenario,'ClassID',1);
lead = vehicle(scenario,'ClassID',1,'Position',[D0 -1.8 0]);
trajectory(ego,[0 -1.8 0;400 -1.8 0],v);
radar = drivingRadarDataGenerator('SensorIndex',1,'MountingLocation',[1.2 0 0], ...
    'RangeLimits',[0 150],'UpdateRate',10);
restart(scenario); T=[]; Dt=[]; Dm=[];
while advance(scenario)
    t = scenario.SimulationTime; poses = targetPoses(ego);
    dTrue = poses(1).Position(1);
    if dTrue < 2, break; end
    [dets,numDet] = radar(poses,t);
    if numDet>0, dM = dets{1}.Measurement(1); else, dM = NaN; end
    T(end+1)=t; Dt(end+1)=dTrue; Dm(end+1)=dM;
end
tFirst = T(1);
ix = find(Dt < Dtrig, 1);
tTruth = T(ix-1) + (Dtrig - Dt(ix-1)) / (Dt(ix) - Dt(ix-1)) * (T(ix) - T(ix-1));

jitter = zeros(Nmc,1);
for n = 1:Nmc
    meas = Dm + 3*randn(size(Dm));
    iT = find(meas < Dtrig, 1);
    if isempty(iT), jitter(n) = NaN; continue; end
    jitter(n) = T(iT) - tTruth;
end
jitter(isnan(jitter)) = [];

% ---- 碰撞面 ----
Crash = zeros(numel(dList), numel(sigList));
for di = 1:numel(dList)
    for si = 1:numel(sigList)
        c = 0;
        for n = 1:Nmc
            j = 0; if si==2, j = jitter(randi(numel(jitter))); end
            tB = tFirst + dList(di) + j;
            if tB >= T(end), c = c+1; continue; end
            if interp1(T,Dt,tB) - S < 0, c = c+1; end
        end
        Crash(di,si) = c;
    end
end
rate = 100*Crash/Nmc;
fprintf('碰撞率/次数 (100 km/h, N=%d/格, τ=0):\n', Nmc);
fprintf('  Δt(s)    σ=0        σ=3        CP上端(σ=3)\n');
for di = 1:numel(dList)
    ub = betaincinv(0.975, Crash(di,2)+1, Nmc-Crash(di,2));
    fprintf('  %.3f   %5d(%5.1f%%)  %5d(%5.1f%%)   %.4f\n', dList(di), ...
        Crash(di,1), rate(di,1), Crash(di,2), rate(di,2), ub);
end
% 平台结构输出
fprintf('\n平台检测(σ=3): ');
for di = 2:numel(dList)
    if abs(rate(di,2)-rate(di-1,2)) > 2.5
        fprintf('跳变在 Δt≈%.3f s → %.1f%%; ', dList(di), rate(di,2));
    end
end
fprintf('\n');

% ---- Figure 7 ----
figure('Color','w','Position',[100 100 820 480]);
bh = bar(dList, rate, 'grouped');
bh(1).FaceColor = [0.20 0.40 0.80]; bh(2).FaceColor = [0.85 0.33 0.10];
legend({'σ=0 (参照)','σ=3 m'},'Location','northwest','FontSize',11);
xlabel('\Delta t (s)','FontSize',12); ylabel('碰撞率 (%)','FontSize',12);
grid on; ylim([0 110]); title('100 km/h 联合预算碰撞概率面 (N=1000/格)','FontSize',12);
exportgraphics(gcf, fullfile(OUT,'fig_joint_budget.png'),'Resolution',300);
fprintf('Figure 7 已保存: %s\n', fullfile(OUT,'fig_joint_budget.png'));