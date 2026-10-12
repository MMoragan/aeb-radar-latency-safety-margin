%% D6: 收官修复 —— 偏置符号核验 + 网格鲁棒触发统计 + 噪声-延迟联合预算 + 摩擦敏感性 + 空道虚警
% 对应评审: #1偏置口径 / #2 CP公式与原始计数 / #3 噪声-延迟联合通道 / #6 摩擦敏感性 / #7 空道虚警
clear; clc; close all; rng('default');
base = 'C:\deepseek\论文\文件列表\';
a = 8.0; vList = [60 80 100]; D0 = 60; Dtrig = 40;
sigmaList = [0 1 3]; Nmc = 1000;
v0list = vList/3.6;
stopDist = @(v,tau) v*tau - a*tau^2/6 + (v - a*tau/2)^2/(2*a);

%% 场景 (与 d5 相同, 三种车速)
Tcell={}; Dtcell={}; Dmcell={};
for k = 1:numel(vList)
    v0 = v0list(k);
    scenario = drivingScenario('SampleTime',0.1,'StopTime',10);
    road(scenario,[0 0;400 0],'Lanes',lanespec(2));
    ego  = vehicle(scenario,'ClassID',1);
    lead = vehicle(scenario,'ClassID',1,'Position',[D0 -1.8 0]);
    trajectory(ego,[0 -1.8 0;400 -1.8 0],v0);
    radar = drivingRadarDataGenerator('SensorIndex',1,'MountingLocation',[1.2 0 0], ...
        'RangeLimits',[0 150],'UpdateRate',10);
    restart(scenario);
    T=[]; Dt=[]; Dm=[];
    while advance(scenario)
        t = scenario.SimulationTime;
        poses = targetPoses(ego);
        dTrue = poses(1).Position(1);
        if dTrue < 2, break; end
        [dets,numDet] = radar(poses, t);
        if numDet > 0, dMeas = dets{1}.Measurement(1); else, dMeas = NaN; end
        T(end+1)=t; Dt(end+1)=dTrue; Dm(end+1)=dMeas;
    end
    Tcell{k}=T; Dtcell{k}=Dt; Dmcell{k}=Dm;
end

%% A) 偏置符号核验 (方法节界定依据: 负 = 读数偏短)
fprintf('===== A) 偏置符号核验 =====\n');
for k = 1:numel(vList)
    fprintf('v=%d km/h: 每帧 mean(Dm-Dt) = %+.2f m  (负值=测量读数比真值短)\n', ...
        vList(k), mean(Dmcell{k}-Dtcell{k},'omitnan'));
end

%% B) 网格鲁棒触发统计 (真值连续穿越时刻为基准) + 等效延迟
k80 = 2; T=Tcell{k80}; Dt=Dtcell{k80}; Dm=Dmcell{k80};
iDet = find(~isnan(Dm),1);
% 真值连续穿越 Dtrig 的时刻 (线性插值; Dt 单调递减故用逆序)
ix = find(Dt < Dtrig, 1);
frac = (Dt(ix-1)-Dtrig)/(Dt(ix-1)-Dt(ix));
tTruth = T(ix-1) + frac*(T(ix)-T(ix-1));

M = zeros(Nmc,3); early = zeros(1,3); late = zeros(1,3);
crashC = zeros(1,3); eqTab = zeros(Nmc,1); kEq = 0;
for s = 1:3
    sig = sigmaList(s);
    for n = 1:Nmc
        meas = Dm + sig*randn(size(Dm));
        iT = find(meas < Dtrig, 1);
        if isempty(iT)
            M(n,s) = -stopDist(v0list(2),0); crashC(s) = crashC(s)+1; continue;
        end
        tb = T(iT);
        D_av = interp1(T,Dt,tb);
        M(n,s) = D_av - stopDist(v0list(2),0);
        if M(n,s) < 0, crashC(s) = crashC(s)+1; end
        if tb < tTruth-0.05, early(s) = early(s)+1; end
        if tb > tTruth+0.05, late(s) = late(s)+1; end
        if sig == 3, kEq = kEq+1; eqTab(kEq) = tb - T(iDet); end
    end
end
p_anova = anova1(M, {'sigma_0','sigma_1','sigma_3'}, 'off');
fprintf('\n===== B) 网格鲁棒触发统计 (N=1000, 真值插值穿越 tTruth=%.3fs) =====\n', tTruth);
rowsN=[];
for s = 1:3
    hi = betaincinv(0.975, crashC(s)+1, Nmc-crashC(s));
    lo = betaincinv(0.025, crashC(s)+1, Nmc-crashC(s));
    se = std(M(:,s))/sqrt(Nmc);
    fprintf('σ=%d: 裕度mean=%.2f std=%.2f | 碰撞%d次 | CP95[%.4f,%.4f] | 早触发%.1f%% 晚触发%.1f%%\n', ...
        sigmaList(s), mean(M(:,s)), std(M(:,s)), crashC(s), hi, lo, 100*early(s)/Nmc, 100*late(s)/Nmc);
    rowsN=[rowsN; sigmaList(s) mean(M(:,s)) std(M(:,s)) ...
        mean(M(:,s))-1.96*se mean(M(:,s))+1.96*se crashC(s) hi 100*early(s)/Nmc 100*late(s)/Nmc];
end
writetable(array2table(rowsN,'VariableNames', ...
    {'sigma_m','mean_margin_m','std_m','meanCI_lo','meanCI_hi','crash_count','cp95_upper','early_pct','late_pct'}), ...
    fullfile(base,'results_noise_mc_v2.csv'));
eq = eqTab(1:kEq);
fprintf('σ=3 等效延迟(触发时刻-首检时刻): median=%.2fs  p95=%.2fs  max=%.2fs\n', ...
    median(eq), quantile(eq,0.95), max(eq));
fprintf('联合预算(100km/h瞬时制动): 0.40s - p95(%.2fs) = %.2fs\n', quantile(eq,0.95), 0.4-quantile(eq,0.95));
fprintf('ANOVA p = %.5f\n', p_anova);

%% C) 摩擦/减速度敏感性 (纯解析: a=5湿路, 6.5, 8干路; τ=0 与 0.3)
fprintf('\n===== C) 摩擦/减速度敏感性 (临界延迟s; 湿路 a=5, 中间 6.5, 干路 8) =====\n');
aList = [5 6.5 8]; tauList2 = [0 0.3]; dGrid = 0:0.05:1.5;
rowsF=[];
for tau = tauList2
    for v = vList
        vv = v/3.6; str = sprintf('%d km/h (τ=%.1f): ', v, tau);
        for aF = aList
            S = vv*tau - aF*tau^2/6 + (vv - aF*tau/2)^2/(2*aF);
            if 60 - vv*0.1 - S < 0
                str = [str sprintf(' a=%.1f→无法避免', aF)];
                rowsF=[rowsF; v aF tau NaN]; continue;
            end
            crit = NaN;
            for t = dGrid
                if 60 - vv*(t+0.1) - S < 0, crit = t; break; end
            end
            if isnan(crit), str = [str sprintf(' a=%.1f→>1.5', aF)];
            else, str = [str sprintf(' a=%.1f→%.2f', aF, crit)]; end
            rowsF=[rowsF; v aF tau crit];
        end
        disp(str);
    end
end
writetable(array2table(rowsF,'VariableNames', ...
    {'v_kmh','a_mps2','tau_s','critical_delay_s'}), fullfile(base,'results_friction.csv'));

%% D) 空道虚警 MC + 解析误触发概率
fprintf('\n===== D) 空道虚警 MC + 解析误触发概率 =====\n');
scenario = drivingScenario('SampleTime',0.1,'StopTime',10);
road(scenario,[0 0;400 0],'Lanes',lanespec(2));
ego = vehicle(scenario,'ClassID',1);
trajectory(ego,[0 -1.8 0;400 -1.8 0],80/3.6);
radar = drivingRadarDataGenerator('SensorIndex',1,'MountingLocation',[1.2 0 0], ...
    'RangeLimits',[0 150],'UpdateRate',10);
restart(scenario); Dm2 = [];
while advance(scenario)
    [dets,numDet] = radar(targetPoses(ego), scenario.SimulationTime);
    if numDet > 0, Dm2(end+1) = dets{1}.Measurement(1); end
end
trig = 0;
for n = 1:Nmc
    if any(Dm2 + 3*randn(size(Dm2)) < Dtrig), trig = trig + 1; end
end
fprintf('空道(无目标) σ=3, N=%d: 虚警触发 %d 次 (合成雷达无杂波模型 → 0, 见 Discussion 局限)\n', Nmc, trig);
fprintf('解析: 目标在阈值上方 r 米时的单帧误触发概率 P = Φ((r-1.1)/σ), σ=3:\n');
for r = 2:2:8
    fprintf('  r=%d m → P=%.1f%%\n', r, 100*normcdf((r-1.1)/3));
end

fprintf('\n全部输出: results_noise_mc_v2.csv / results_friction.csv + 终端数字\n');