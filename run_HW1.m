%% run_HW1.m
%  UUM 523E - HW-1 (Group 2). Odevin istedigi tum sonuclari F16_6DOF.slx'ten uretir.
%
%   1. Problem 1 - Tablo 1: Aerodynamics alt sistemi, 10 durum -> AeroResults (10x12)
%   2. Problem 1 - Tablo 2: Propulsion alt sistemi, 5 durum   -> ThrustResults (5x1)
%   3. Sonuc dosyasi: HW1_Group2.mat (yalnizca AeroResults ve ThrustResults)
%   4. Problem 2a: 5 kosulda C_L ve C_D - alpha grafikleri
%   5. Problem 2b: 9 beta icin C_m,T - alpha ve C_m,alpha - alpha grafikleri
%
%  Alt sistemler gecici test modellerine (harness) kopyalanip tek basina
%  calistirilir; F16_6DOF.slx DEGISTIRILMEZ. Grafikler figures/ klasorune kaydedilir.
%  Dogrulama (bagimsiz referansla karsilastirma): test_aero_table1, test_thrust_table2.
%
%  Calistirma:  >> run_HW1

here = fileparts(mfilename('fullpath'));
run(fullfile(here, 'init_F16.m'));
groupNo = 2;
figDir  = fullfile(here, 'figures');
if ~isfolder(figDir), mkdir(figDir); end
load_system(fullfile(here, 'F16_6DOF.slx'));

%% Ucus kosulu: 0.4 Mach, 10000 ft (Stevens-Lewis-Johnson App. A.6, SI)
h     = h_val_m;
Tfac  = 1 - lapse_SI*h;
Tatm  = T0_SI*Tfac;   if h >= h_trop_SI, Tatm = T_iso_SI; end
rho   = R0_SI*Tfac^4.14;
V     = Mach_val*sqrt(gamma_R_SI*Tatm);
qbar  = 0.5*rho*V^2;
fprintf('Ucus kosulu: M = %.2f, h = %.0f m -> V = %.4f m/s, qbar = %.3f Pa\n', Mach_val, h, V, qbar);

% Aerodynamics giris sirasi (port sirasi) ve sabit girisler
aeroIn = {'alpha','beta','dh','dlef','q','d_sb','da','dr','r','p', ...
          'x_cg','q_dyn','S_ref','b_ref','c_ref','TAS'};
base = struct('alpha',0, 'beta',0, 'dh',0, 'dlef',0, 'q',0, 'd_sb',0, 'da',0, 'dr',0, ...
              'r',0, 'p',0, 'x_cg',xcg, 'q_dyn',qbar, 'S_ref',S, 'b_ref',b, 'c_ref',cbar, 'TAS',V);
aeroTm = buildAeroHarness('F16_6DOF/Aerodynamics', aeroIn);

%% 1. Problem 1 - Tablo 1 (aero dogrulamasi; dlef tablodan, LEF tarifesi yok)
%        alpha  beta    p      q     r     da      dh     dr    dlef  dsb   [deg, deg/s]
tab1 = [  0     0       0      0     0     0       0      0     0     0;
          0     0       0      0     0     0       0      0     0     60;
          20    0       0      0     0     0       0      0     0     0;
          0     12.5    0      0     0     0       0      0     0     0;
          0     0       0      0     0     0      -15     0     0     0;
          0     0       0      0     0     0       0      0     25    0;
          0     0       0      0     0     10      0      0     0     0;
          0     0       0      0     0     0       0     -15    0     0;
          5.16  0       0      0     0     0      -0.47   0     7.55  0;
          5.95  0      -0.53   1.21  2.09 -0.055  -0.62  -0.33  8.65  0 ];
U = base;
U.alpha = tab1(:,1);  U.beta = tab1(:,2);
U.p  = tab1(:,3)*pi/180;  U.q  = tab1(:,4)*pi/180;  U.r = tab1(:,5)*pi/180;   % rad/s
U.da = tab1(:,6);  U.dh = tab1(:,7);  U.dr = tab1(:,8);  U.dlef = tab1(:,9);  U.d_sb = tab1(:,10);
[C, FM] = aeroEval(aeroTm, U, aeroIn);
AeroResults = [C, FM];                     % CX CY CZ Cl Cm Cn X Y Z L M N

names = {'CX','CY','CZ','Cl','Cm','Cn','X [N]','Y [N]','Z [N]','L [Nm]','M [Nm]','N [Nm]'};
fprintf('\nProblem 1 - Tablo 1 (AeroResults):\n%4s', 'case'); fprintf('%11s', names{1:6}); fprintf('\n');
for i = 1:10, fprintf('%4d', i); fprintf('%11.6f', AeroResults(i,1:6)); fprintf('\n'); end
fprintf('%4s', 'case'); fprintf('%13s', names{7:12}); fprintf('\n');
for i = 1:10, fprintf('%4d', i); fprintf('%13.2f', AeroResults(i,7:12)); fprintf('\n'); end

%% 2. Problem 1 - Tablo 2 (itki dogrulamasi; kararli hal)
%        Mach   irtifa [ft]  gaz kolu [%]
tab2 = [ 0.25    5000        10;
         0.65   25000        77;
         0.90   45000        95;
         0.40   10000        16.66;
         0.40   10000        19.77];
ThrustResults = thrustEval('F16_6DOF/Propulsion', tab2, ft2m, thr_BP, P1_tab);

fprintf('\nProblem 1 - Tablo 2 (ThrustResults):\n%4s %6s %8s %8s %14s\n', 'case','Mach','h [ft]','thr [%]','T [N]');
for i = 1:5
    fprintf('%4d %6.2f %8d %8.2f %14.4f\n', i, tab2(i,1), tab2(i,2), tab2(i,3), ThrustResults(i));
end

%% 3. Sonuc dosyasi
resFile = fullfile(here, sprintf('HW1_Group%d.mat', groupNo));
save(resFile, 'AeroResults', 'ThrustResults');
fprintf('\nSonuc dosyasi kaydedildi: %s\n', resFile);

%% 4. Problem 2a - C_L ve C_D (beta, dh, dlef, dsb kosullari)
%           beta  dh    dlef  dsb  [deg]      etiket
cond2a = { 0,    0,    0,    0,   'i: baseline';
           0,    0,    0,    60,  'ii: \delta_{sb} = 60^\circ';
           0,    0,    25,   0,   'iii: \delta_{lef} = 25^\circ';
           15,   0,    0,    0,   'iv: \beta = 15^\circ';
           0,   -25,   0,    0,   'v: \delta_h = -25^\circ'};
alphaA = (-20:0.5:90)';
CL = zeros(numel(alphaA), size(cond2a,1));  CD = CL;
for j = 1:size(cond2a,1)
    U = base;
    U.alpha = alphaA;  U.beta = cond2a{j,1};  U.dh = cond2a{j,2};
    U.dlef  = cond2a{j,3};  U.d_sb = cond2a{j,4};
    C = aeroEval(aeroTm, U, aeroIn);
    a = alphaA*pi/180;  bt = cond2a{j,1}*pi/180;
    CL(:,j) =  C(:,1).*sin(a) - C(:,3).*cos(a);                              % ruzgar ekseni
    CD(:,j) = -C(:,1).*cos(a)*cos(bt) - C(:,2)*sin(bt) - C(:,3).*sin(a)*cos(bt);
end

fig = figure('Name','Problem 2a', 'Position',[100 100 1100 420]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing','compact');
nexttile(tl);  plot(alphaA, CL, 'LineWidth',1.2);  lefMark();  grid on;
xlabel('\alpha [deg]');  ylabel('C_L');  title('Lift coefficient');
nexttile(tl);  plot(alphaA, CD, 'LineWidth',1.2);  lefMark();  grid on;
xlabel('\alpha [deg]');  ylabel('C_D');  title('Drag coefficient');
legend(cond2a(:,5), 'Location','northwest');
exportgraphics(fig, fullfile(figDir, 'P2a_CL_CD.png'), 'Resolution',200);

%% 5. Problem 2b - C_m,T ve C_m,alpha (x_cg = 0.35 cbar, dh = dlef = dsb = 0, q = 0)
beta2b = [-30 -15 -10 -6 0 6 10 15 30];
Cm = zeros(numel(alphaA), numel(beta2b));
for j = 1:numel(beta2b)
    U = base;  U.alpha = alphaA;  U.beta = beta2b(j);
    C = aeroEval(aeroTm, U, aeroIn);
    Cm(:,j) = C(:,5);
end
inB  = alphaA >= -20 & alphaA <= 60;
aB   = alphaA(inB);
Cma  = zeros(numel(aB), numel(beta2b));
for j = 1:numel(beta2b)
    Cma(:,j) = gradient(Cm(inB,j), aB);                                      % 1/deg
end
leg2b = arrayfun(@(x) sprintf('\\beta = %g^\\circ', x), beta2b, 'UniformOutput', false);

fig = figure('Name','Problem 2b - Cm', 'Position',[100 100 780 460]);
betaPlot(alphaA, Cm, beta2b);  lefMark();  grid on;
xlabel('\alpha [deg]');  ylabel('C_{m,T}');
title('Pitching moment coefficient (x_{cg} = 0.35 c, \delta_h = \delta_{lef} = \delta_{sb} = 0)');
legend(leg2b, 'Location','eastoutside');
exportgraphics(fig, fullfile(figDir, 'P2b_Cm.png'), 'Resolution',200);

fig = figure('Name','Problem 2b - Cm_alpha', 'Position',[100 100 780 460]);
betaPlot(aB, Cma, beta2b);  lefMark();  grid on;  yline(0, 'k-', 'HandleVisibility','off');
xlabel('\alpha [deg]');  ylabel('C_{m,\alpha} [1/deg]');
title('Pitch stiffness  C_{m,\alpha} = \partial C_{m,T} / \partial\alpha');
legend(leg2b, 'Location','eastoutside');
exportgraphics(fig, fullfile(figDir, 'P2b_Cm_alpha.png'), 'Resolution',200);

% Kisa ozet (rapordaki yorum icin): C_m,alpha > 0 -> statik olarak kararsiz
fprintf('\nProblem 2b - C_m,alpha > 0 (kararsiz) olan alpha araliklari [deg]:\n');
for j = 1:numel(beta2b)
    fprintf('  beta = %5.1f : %s\n', beta2b(j), ranges(aB, Cma(:,j) > 0));
end

close_system(aeroTm, 0);
evalin('base', 'clear in_*');
fprintf('\nGrafikler: %s\n', figDir);

%% Yardimci fonksiyonlar
function tm = buildAeroHarness(src, inNames)
% Aerodynamics'i gecici bir modele kopyalar. Her giris bir From Workspace blogundan
% gelir (in_<ad> = [t deger]); her zaman adimi bir durumdur (ayrik, adim = 1).
tm = 'aero_hw1_harness';
if bdIsLoaded(tm), close_system(tm, 0); end
new_system(tm);
add_block(src, [tm '/Aero'], 'Position', [300 50 450 650]);
ah = get_param([tm '/Aero'], 'PortHandles');
for k = 1:numel(inNames)
    y = 40 + 37*k;
    blk = add_block('simulink/Sources/From Workspace', [tm '/in_' inNames{k}], ...
            'VariableName', ['in_' inNames{k}], 'Interpolate', 'off', 'SampleTime', '1', ...
            'OutputAfterFinalValue', 'Holding final value', 'Position', [100 y 200 y+20]);
    ph = get_param(blk, 'PortHandles');
    add_line(tm, ph.Outport(1), ah.Inport(k));
end
for k = 1:2
    ob = add_block('simulink/Sinks/Out1', [tm '/y' num2str(k)], ...
            'Position', [550 200+200*(k-1) 580 214+200*(k-1)]);
    oph = get_param(ob, 'PortHandles');
    add_line(tm, ah.Outport(k), oph.Inport(1));
end
set_param(tm, 'SolverType','Fixed-step', 'Solver','FixedStepDiscrete', 'FixedStep','1', ...
              'SaveTime','off', 'SaveOutput','on', 'OutputSaveName','yout', ...
              'SaveFormat','Array', 'ReturnWorkspaceOutputs','on');
end

function [C, FM] = aeroEval(tm, U, inNames)
% U: her giris icin skaler ya da N'lik sutun. C = [CX CY CZ Cl Cm Cn], FM = [X Y Z L M N]
N = max(structfun(@numel, U));
t = (0:N-1)';
for k = 1:numel(inNames)
    v = U.(inNames{k});
    if isscalar(v), v = repmat(v, N, 1); end
    assignin('base', ['in_' inNames{k}], [t v(:)]);
end
set_param(tm, 'StopTime', num2str(N-1));
so = sim(tm);
FM = so.get('yout');                       % N x 6: [F_aero M_aero]
qS = U.q_dyn .* U.S_ref;
C  = [FM(:,1:3)./qS, FM(:,4)./(qS.*U.b_ref), FM(:,5)./(qS.*U.c_ref), FM(:,6)./(qS.*U.b_ref)];
end

function T = thrustEval(src, cases, ft2m, thr_BP, P1_tab)
% Propulsion'i her durum icin t = 0'da calistirir; motor kararli halde (P3_0 = P1).
tm = 'thrust_hw1_harness';
if bdIsLoaded(tm), close_system(tm, 0); end
new_system(tm);
add_block(src, [tm '/Engine'], 'Position', [300 50 420 200]);
inNames = {'thr','mach','alt_m'};
ah = get_param([tm '/Engine'], 'PortHandles');
for k = 1:3
    y = 40 + 50*k;
    blk = add_block('simulink/Sources/Constant', [tm '/u_' inNames{k}], ...
            'Value', ['u_' inNames{k}], 'Position', [120 y 200 y+20]);
    ph = get_param(blk, 'PortHandles');
    add_line(tm, ph.Outport(1), ah.Inport(k));
end
ob = add_block('simulink/Sinks/Out1', [tm '/y'], 'Position', [500 115 530 129]);
oph = get_param(ob, 'PortHandles');
add_line(tm, ah.Outport(1), oph.Inport(1));
set_param(tm, 'StopTime','0', 'SaveOutput','on', 'OutputSaveName','yout', ...
              'SaveFormat','Array', 'ReturnWorkspaceOutputs','on');
T = zeros(size(cases,1), 1);
P3_0_old = evalin('base', 'P3_0');
for i = 1:size(cases,1)
    assignin('base', 'u_thr',   cases(i,3));
    assignin('base', 'u_mach',  cases(i,1));
    assignin('base', 'u_alt_m', cases(i,2)*ft2m);
    assignin('base', 'P3_0',    interp1(thr_BP, P1_tab, cases(i,3)));   % P3 = P1
    so = sim(tm);
    yo = so.get('yout');
    T(i) = yo(end, 1);
end
close_system(tm, 0);
assignin('base', 'P3_0', P3_0_old);
evalin('base', 'clear u_thr u_mach u_alt_m');
end

function betaPlot(x, Y, beta)
% +-beta cifti ayni renk (simetri gorulsun): beta < 0 kesikli, beta > 0 duz, beta = 0 kalin siyah
mags = unique(abs(beta(beta ~= 0)));
cmap = turbo(numel(mags) + 2);  cmap = cmap(2:end-1, :);     % uclardaki koyu renkler atlanir
hold on;
for j = 1:numel(beta)
    if beta(j) == 0
        plot(x, Y(:,j), 'k-', 'LineWidth', 2.2);
        continue;
    end
    c = cmap(mags == abs(beta(j)), :);
    if beta(j) < 0, ls = '--'; else, ls = '-'; end
    plot(x, Y(:,j), ls, 'Color', c, 'LineWidth', 1.3);
end
hold off;
end

function lefMark()
% LEF tablolari alpha <= 45 deg'e kadar; ustunde lookup'lar Clip ile sabit tutar
xline(45, '--', 'LEF data limit', 'Color',[0.4 0.4 0.4], 'HandleVisibility','off', ...
      'LabelVerticalAlignment','bottom');
end

function s = ranges(x, mask)
% mantiksal maskenin dogru oldugu araliklari '[a, b]' metni olarak verir
s = '';
d = diff([false; mask(:); false]);
i1 = find(d == 1);  i2 = find(d == -1) - 1;
for k = 1:numel(i1)
    s = [s sprintf('[%g, %g] ', x(i1(k)), x(i2(k)))]; %#ok<AGROW>
end
if isempty(s), s = 'yok'; end
end
