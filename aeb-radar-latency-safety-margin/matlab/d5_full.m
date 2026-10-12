%% D5: 升级版主实验 —— 全矩阵 + 制动建立时间灵敏度 + MC(1000)+CI + 误触发 + 1.5s扫描
% 针对评审意见逐条升级:
%  [1] MC 提升到 N=1000, 碰撞率给 Clopper-Pearson 95% 置信区间
%  [2] 制动建立时间(线性爬升 tau)灵敏度, 输出"临界延迟 vs tau"表
%  [3] 误触发分析: 范围触发下"提前触发(虚警倾向)"比例
%  [4] 延迟扫描扩到 1.5 s, 解决 Table 2 ">=0.8 外推"问题
%  [5] 明确两种触发模型口径(模型A时间触发/模型B范围触发)
clear; clc; close all;
rng('default');
base = 'C:\deepseek\论文\文件列表\';

%% 参数
a        = 8.0;                            % 满减速度 m/s^2
vList    = [60 80 100];                    % km/h
dList    = [0 0.2 0.4 0.6 0.8 1.0 1.2 1.5]; % 检测延迟 s (扩到1.5)
tauList  = [0 0.3 0.5];                    % 制动建立时间(线性爬升) s
D0       = 60;                             % 前车初始间距 m
Dtrig    = 40;                             % 范围触发阈值 m
sigmaList= [0 1 3];                        % 测距噪声 s.m
Nmc      = 1000;

% 制动距离: 前 tau 秒线性爬升至 a, 之后恒定 a
%   S(v) = v*tau - a*tau^2/6 + (v - a*tau/2)^2/(2a)   (tau=0 时退化为 v^2/2a)
stopDist = @(v, tau) v*tau - a*tau^2/6 + (v - a*tau/2)^2/(2*a);

%% 场景: 每种车速跑一次, 记录 真值距离 Dt / 雷达测量 Dm
Tcell = {}; Dtcell = {}; Dmcell = {};
for k = 1:numel(vList)
    v0 = vList(k)/3.6;
    scenario = drivingScenario('SampleTime',0.1,'StopTime',10);
    road(scenario,[0 0;400 0],'Lanes',lanespec(2));
    ego  = vehicle(scenario,'ClassID',1);
    lead = vehicle(scenario,'ClassID',1,'Position',[D0 -1.8 0]);
    trajectory(ego,[0 -1.8 0;400 -1.8 0],v0);
    radar = drivingRadarDataGenerator('SensorIndex',1, ...
        'MountingLocation',[1.2 0 0],'RangeLimits',[0 150],'UpdateRate',10);
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

%% ===== 实验1(模型A: 时间触发, 首检+延迟, 含一个采样周期0.1s) =====
rows = [];
for k = 1:numel(vList)
    v0 = vList(k)/3.6;
    T = Tcell{k}; Dt = Dtcell{k}; Dm = Dmcell{k};
    iDet = find(~isnan(Dm),1);
    for j = 1:numel(dList)
        dt = dList(j);
        tBrake = T(iDet) + dt;
        if tBrake >= T(end)
            D_avail = 0;
        else
            D_avail = interp1(T,Dt,tBrake);
        end
        S = stopDist(v0,0);
        margin = D_avail - S;
        ttc = D_avail/v0;
        rows = [rows; vList(k) dt tBrake D_avail S margin ttc (margin<0)];
    end
end
TblA = array2table(rows,'VariableNames', ...
    {'v_kmh','delay_s','t_brake_s','D_avail_m','stop_dist_m','margin_m','TTC_s','crash'});
writetable(TblA, fullfile(base,'results_latency_v2.csv'));
fprintf('保存: results_latency_v2.csv (延迟扩到1.5s)\n');

%% ===== 实验1b: 临界延迟 vs 制动建立时间 (灵敏度分析) =====
critRows = [];
for k = 1:numel(vList)
    v0 = vList(k)/3.6;
    T = Tcell{k}; Dt = Dtcell{k}; Dm = Dmcell{k};
    iDet = find(~isnan(Dm),1);
    for tau = tauList
        crit = NaN;
        for j = 1:numel(dList)
            dt = dList(j);
            tBrake = T(iDet) + dt;
            if tBrake >= T(end)
                D_avail = 0;
            else
                D_avail = interp1(T,Dt,tBrake);
            end
            if D_avail - stopDist(v0,tau) < 0
                crit = dt; break;
            end
        end
        critRows = [critRows; vList(k) tau crit];
    end
end
TblB = array2table(critRows,'VariableNames',{'v_kmh','tau_s','critical_delay_s'});
writetable(TblB, fullfile(base,'results_critical_tau.csv'));
disp('===== 临界延迟 vs 制动建立时间 tau (s) =====');
disp(TblB);

%% ===== 实验2(模型B: 范围触发, 测量<40m): MC=1000 + CI + 误触发, v0=80 =====
k80 = find(vList==80);
T = Tcell{k80}; Dt = Dtcell{k80}; Dm = Dmcell{k80};
v0 = 80/3.6;
tTruth = T(find(Dt < Dtrig,1));          % 真值越过阈值时刻(基准时刻)

M = zeros(Nmc, numel(sigmaList));
early = zeros(1,numel(sigmaList)); late = zeros(1,numel(sigmaList));
crashCount = zeros(1,numel(sigmaList));
for s = 1:numel(sigmaList)
    sig = sigmaList(s);
    for n = 1:Nmc
        meas = Dm + sig*randn(size(Dm));
        iTrig = find(meas < Dtrig, 1);
        if isempty(iTrig)
            M(n,s) = -stopDist(v0,0);   % 未触发 -> 必撞
            crashCount(s) = crashCount(s) + 1;
            late(s) = late(s) + 1;
            continue
        end
        tBrake = T(iTrig);
        D_avail = interp1(T,Dt,tBrake);
        M(n,s) = D_avail - stopDist(v0,0);
        if M(n,s) < 0, crashCount(s) = crashCount(s) + 1; end
        if tBrake < tTruth - 0.05, early(s) = early(s) + 1; end   % 提前触发(虚警倾向)
        if tBrake > tTruth + 0.05, late(s)  = late(s)  + 1; end
    end
end
p_anova = anova1(M, {'sigma_0','sigma_1','sigma_3'}, 'off');

rowsN = [];
for s = 1:numel(sigmaList)
    m = M(:,s); kc = crashCount(s);
    lo = betaincinv(0.025, kc+1,   Nmc-kc);    % Clopper-Pearson 95%下界
    hi = betaincinv(0.975, kc+1,   Nmc-kc);    % 95%上界
    rowsN = [rowsN; sigmaList(s) mean(m) std(m) ...
             mean(m)-1.96*std(m)/sqrt(Nmc) mean(m)+1.96*std(m)/sqrt(Nmc) ...
             kc hi 100*early(s)/Nmc 100*late(s)/Nmc];
end
TblN = array2table(rowsN,'VariableNames', ...
    {'sigma_m','mean_margin_m','std_m','meanCI_lo','meanCI_hi', ...
     'crash_count','crashCI_upper','early_trigger_pct','late_trigger_pct'});
writetable(TblN, fullfile(base,'results_noise_mc.csv'));
fprintf('\nANOVA p (噪声对裕度影响显著性) = %.4f\n', p_anova);
disp(TblN);

%% ===== 图1: 裕度 vs 延迟 (扩展, 3车速, tau=0) =====
figure(1); hold on; grid on;
for k = 1:numel(vList)
    vv = vList(k);
    sub = TblA(TblA.v_kmh==vv,:);
    plot(sub.delay_s, sub.margin_m,'-o','LineWidth',1.5);
end
xlabel('Detection delay \Deltat (s)'); ylabel('Safety margin (m)');
legend(arrayfun(@(v)sprintf('%d km/h',v),vList,'UniformOutput',false));
title('AEB Safety Margin vs Detection Latency (extended sweep)');
yline(0,'r--','Collision');
exportgraphics(gcf, fullfile(base,'fig_margin_vs_delay_v2.png'));

%% ===== 图2: 临界延迟 vs 制动建立时间 =====
figure(2); hold on; grid on;
for k = 1:numel(vList)
    vv = vList(k);
    sub = TblB(TblB.v_kmh==vv,:);
    plot(sub.tau_s, sub.critical_delay_s,'-s','LineWidth',1.5);
end
xlabel('Braking build-up time \tau (s)'); ylabel('Critical detection latency (s)');
legend(arrayfun(@(v)sprintf('%d km/h',v),vList,'UniformOutput',false));
title('Critical Latency vs Braking Build-up Time');
exportgraphics(gcf, fullfile(base,'fig_critical_vs_tau.png'));

%% ===== 图3: 噪声MC箱线图 =====
figure(3);
boxplot(M, {'sigma=0 m','sigma=1 m','sigma=3 m'});
ylabel('Safety margin (m)');
title(sprintf('Range-noise Monte Carlo (N=%d), 80 km/h', Nmc));
hold on; plot(1:3, mean(M,1),'r*','MarkerSize',10);
exportgraphics(gcf, fullfile(base,'fig_noise_mc.png'));

%% 内部一致性核验
v60 = 60/3.6;
fprintf('\n一致性核验: 60km/h delay=0 裕度 = %.2f  (解析期望 60-v*0.1-v^2/2a = %.2f)\n', ...
    TblA.margin_m(1), 60 - v60*0.1 - v60^2/(2*a));
fprintf('全部输出已保存至: %s\n', base);