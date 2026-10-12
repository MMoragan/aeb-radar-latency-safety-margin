%% fig_export.m — 论文图 1/2/3/4/5 重导出 (300 dpi, 无工具栏; MDPI 要求)
% 图2 裕度-延迟 / 图3 TTC-延迟 / 图4 临界延迟-τ: 解析公式直接重算 (与 Table 1/2 数字勾稽, 秒出)
% 图5 噪声盒须图: 80 km/h 范围触发 MC (N=1000/层, 口径与 Table 4 一致: 无偏置, 40 m 阈值)
% 图1 场景俯视图: 可选图, 顺带生成
clear; clc; close all; rng('default');
OUT = 'C:\deepseek\论文\文件列表';
D0 = 60; a = 8; dtGrid = 0:0.05:1.5; Vkmh = [60 80 100]; V = Vkmh/3.6;

% ---------- Figure 2: safety margin vs delay ----------
figure('Color','w','Position',[60 60 780 480]); hold on;
for i = 1:3
    v = V(i); S = v^2/(2*a);
    m = D0 - v*(0.1+dtGrid) - S;              % 首检 0.1 s 量化延迟计入 (Table 1 口径)
    plot(dtGrid, m, 'LineWidth', 1.8);
end
yline(0, 'r--', 'LineWidth', 1.2);
xlabel('\Delta t (s)','FontSize',12); ylabel('Safety margin (m)','FontSize',12);
legend({'60 km/h','80 km/h','100 km/h'},'Location','southwest','FontSize',11);
grid on; ylim([-36 46]); box on;
exportgraphics(gcf, fullfile(OUT,'fig_margin_vs_delay_v2.png'),'Resolution',300);

% ---------- Figure 3: TTC at brake trigger vs delay ----------
figure('Color','w','Position',[60 60 780 480]); hold on;
for i = 1:3
    TTC = D0/V(i) - 0.1 - dtGrid;             % TTC = D_avail/v = D0/v - 0.1 - dt
    plot(dtGrid, TTC, 'LineWidth', 1.8);
end
xlabel('\Delta t (s)','FontSize',12); ylabel('TTC at brake trigger (s)','FontSize',12);
legend({'60 km/h','80 km/h','100 km/h'},'Location','northeast','FontSize',11);
grid on; ylim([0 4]); box on;
exportgraphics(gcf, fullfile(OUT,'fig_ttc_vs_delay_v2.png'),'Resolution',300);

% ---------- Figure 4: critical latency vs braking build-up tau ----------
tau = 0:0.05:0.5;
figure('Color','w','Position',[60 60 780 480]); hold on;
for i = 1:3
    v = V(i);
    S = v.*tau - a.*tau.^2/6 + (v - a.*tau/2).^2/(2*a);
    dtCrit = (D0 - S)./v - 0.1;               % Table 2 口径
    plot(tau, dtCrit, 'LineWidth', 1.8);
end
xlabel('Braking build-up time \tau (s)','FontSize',12);
ylabel('Critical latency (s)','FontSize',12);
legend({'60 km/h','80 km/h','100 km/h'},'Location','northeast','FontSize',11);
grid on; box on;
exportgraphics(gcf, fullfile(OUT,'fig_critical_vs_tau.png'),'Resolution',300);

% ---------- Figure 5: range-noise boxplot (80 km/h, N=1000) ----------
% 口径同 Table 4: 无偏置; 触发时刻 = 测量信号插值穿越 40 m 的时刻; 裕度 = 该时刻真值距离 - S
% (σ=0 时 tB=0.9 s, D_avail=40 m → 裕度 9.14 m ✓)
v = 80/3.6; S = v^2/(2*a); T = 0.1:0.1:2.0; Dtrue = D0 - v*T;
sig = [0 1 3]; N = 1000; mMat = NaN(N,3);
for si = 1:3
    for n = 1:N
        Dm = Dtrue + sig(si)*randn(size(Dtrue));
        ix = find(Dm < 40, 1);
        if isempty(ix), continue; end
        tB = T(ix-1) + (40 - Dm(ix-1))/(Dm(ix) - Dm(ix-1)) * (T(ix) - T(ix-1));
        mMat(n,si) = interp1(T, Dtrue, tB) - S;
    end
end
figure('Color','w','Position',[60 60 780 480]);
boxplot(mMat, {'\sigma = 0 m','\sigma = 1 m','\sigma = 3 m'},'Widths',0.5,'Colors','k');
hold on;
for si = 1:3, plot(si, mean(mMat(:,si),'omitnan'), 'r*','MarkerSize',14); end
xlabel('Range-noise level','FontSize',12); ylabel('Safety margin (m)','FontSize',12);
grid on; box on;
exportgraphics(gcf, fullfile(OUT,'fig_noise_mc.png'),'Resolution',300);
fprintf('Fig5 means: %.2f / %.2f / %.2f m (expect ~9.14 / 9.34 / 10.08)\n', ...
    mean(mMat(:,1),'omitnan'), mean(mMat(:,2),'omitnan'), mean(mMat(:,3),'omitnan'));

% ---------- Figure 1 (可选): 场景俯视图 ----------
figure('Color','w','Position',[60 60 900 420]); hold on;
rectangle('Position',[0 -0.06 160 0.12],'FaceColor',[0.55 0.55 0.55],'EdgeColor','none');
plot([0 160],[1.8 1.8],'--','Color',[0.45 0.45 0.45],'LineWidth',1.2);
plot([0 160],[-1.8 -1.8],'--','Color',[0.45 0.45 0.45],'LineWidth',1.2);
% 自车 (约 4.4x1.9 m, 位于 x=2)
rectangle('Position',[2 -0.95 4.4 1.9],'FaceColor',[0.20 0.40 0.80],'EdgeColor','k','LineWidth',1.2);
% 静止前车 (x=60)
rectangle('Position',[60 -0.95 4.4 1.9],'FaceColor',[0.85 0.33 0.10],'EdgeColor','k','LineWidth',1.2);
% 雷达视场锥 (半角 8 度, 量程 100 m)
th = 8*pi/180; px = 6.4; py = 0; r = 100;
patch([px px+r*cos(th) px+r*cos(-th)],[py py+r*sin(th) py-r*sin(th)], ...
    [0.9 0.9 0.2],'FaceAlpha',0.18,'EdgeColor','none');
text(2, 3.2, 'ego vehicle, v_0 = 60/80/100 km/h','FontSize',11);
text(58.5, 3.2, 'stationary target (60 m)','FontSize',11);
text(20, -5.5, 'radar FoV (10 Hz, \sigma up to 3 m)','FontSize',10,'Color',[0.5 0.5 0]);
axis equal; axis([-15 130 -16 10]); grid on;
xlabel('x (m)','FontSize',12); ylabel('y (m)','FontSize',12);
exportgraphics(gcf, fullfile(OUT,'fig_scenario_top.png'),'Resolution',300);

fprintf('All figures exported to %s (300 dpi).\n', OUT);