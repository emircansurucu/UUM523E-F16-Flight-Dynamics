%% run_MT.m
%  UUM 523E - Midterm Project (Grup 2). F-16 trimde uçuş, kararlılık
%  parametreleri ve dinamik modlar.
%
%   0. Hazırlık: koşul i, ii, iii için trim ve rüzgâr ekseni doğrusal modeli
%      (HW-2 ile aynı fonksiyonlar), HW2_Group2.mat ile karşılaştırma
%   1. Problem 3: tam 8x8 ve ayrık (A11, A22) modellerin modları
%                 -> Eig_Full, Eig_Lon, Eig_Lat, Modes_Full, Modes_Dec
%   2. Problem 2: kararlılık parametreleri ve boyutsuz türevler
%                 -> StabParams, StabDerivs
%   3. Problem 1: trimden başlayan doğrusal olmayan benzetim
%                 -> Sim_i, Sim_ii, Sim_iii
%   4. Sonuç dosyası: MT_Group2.mat (sadece bu 10 değişken)
%
%  Koşullar (HW-2 Tablo 1): i = durum 1 (düz uçuş, xcg 0.25c),
%  ii = durum 4 (60 deg yatışlı dönüş), iii = durum 6 (düz uçuş, xcg 0.35c).
%  Doğrusal modelde LEF trim değerinde sabittir (ödevde istendiği gibi).
%
%  Çalıştırma:  >> run_MT

here = fileparts(mfilename('fullpath'));
run(fullfile(here, 'init_F16.m'));
groupNo = 2;
figDir  = fullfile(here, 'figures');
load_system(fullfile(here, 'F16_6DOF.slx'));
warning('off', 'Simulink:Engine:InitStatesInArrayFormatShouldBeAvoided');
d2r = pi/180;  r2d = 180/pi;
linName = {'i', 'ii', 'iii'};

%% 0. Hazırlık - trim ve doğrusallaştırma
%          Mach  h [ft]  gamma  beta  phi  nz   dönüş  xcg
caseTab = [ 0.6  10000    0     NaN    0   NaN   0     0.25;
            0.6  10000    0      0    60   NaN   1     0.25;
            0.6  10000    0     NaN    0   NaN   0     0.35 ];

% Uçuş koşulu (Stevens-Lewis-Johnson App. A.6, SI)
for j = 1:3
    h    = caseTab(j,2)*ft2m;
    Tfac = 1 - lapse_SI*h;
    Tatm = T0_SI*Tfac;   if h >= h_trop_SI, Tatm = T_iso_SI; end
    V    = caseTab(j,1)*sqrt(gamma_R_SI*Tatm);
    cases(j).VT    = V;
    cases(j).h     = h;
    cases(j).qps   = 0.5*V^2/(R_ps_SI*Tatm);
    cases(j).gamma = caseTab(j,3)*d2r;
    cases(j).beta  = caseTab(j,4)*d2r;
    cases(j).phi   = caseTab(j,5)*d2r;
    cases(j).nz    = caseTab(j,6);
    cases(j).turn  = caseTab(j,7) == 1;
    cases(j).xcg   = caseTab(j,8);
    cases(j).rho   = R0_SI*Tfac^4.14;
    cases(j).qbar  = 0.5*cases(j).rho*V^2;
end

trims  = cell(3, 1);
A_Wind = zeros(8, 8, 3);  B_Wind = zeros(8, 4, 3);
for xc = [0.25 0.35]
    M = f16_modelInit(xc);
    for j = find(caseTab(:,8) == xc)'
        trims{j} = trim_F16(M, cases(j));
        [A_Wind(:,:,j), B_Wind(:,:,j)] = linearize_F16(M, trims{j}, 'wind');
    end
    F16_6DOF([], [], [], 'term');
end
idx = M.idx;

fprintf('\nHazırlık - trim (deg) ve HW-2 ile karşılaştırma:\n');
fprintf('%5s %9s %8s %8s %8s %8s %8s %8s %8s %9s %10s\n', 'koşul', 'VT[m/s]', 'alpha', ...
        'beta', 'phi', 'theta', 'dh', 'dlef', 'dT[%]', 'J', 'A farkı');
hw2 = load(fullfile(here, 'HW2_Group2.mat'), 'A_Wind');
for j = 1:3
    r  = trims{j}.row;
    eA = norm(A_Wind(:,:,j) - hw2.A_Wind(:,:,j))/norm(hw2.A_Wind(:,:,j));
    fprintf('%5s %9.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %9.2e %10.2e\n', linName{j}, ...
            r(1), r([2 3 4 5 11 14])*r2d, r(10), r(15), eA);
end

%% 1. Problem 3 - Dinamik modlar
%  Satırlar: SP, fugoid, yuvarlanma, spiral, Dutch roll
%  Sütunlar: wn [rad/s], zeta, periyot [s], tau [s], t_yarı [s], t_iki kat [s]
%  Uygulanmayan değerler NaN. Reel kökte tau = -1/lambda, karmaşık kökte
%  tau = -1/Re(lambda) (Lecture 6, s. 5-11). Kararsız kökte tau < 0.
%  Koşul iii'te SP iki reel köke ayrılır; SP satırına kararsız kök yazılır.
modeNames = {'SP', 'fugoid', 'yuvarlanma', 'spiral', 'Dutch roll'};
Eig_Full   = complex(zeros(8, 3));
Eig_Lon    = complex(zeros(4, 3));
Eig_Lat    = complex(zeros(4, 3));
Modes_Full = nan(5, 6, 3);
Modes_Dec  = nan(5, 6, 3);
shareFull  = zeros(8, 3);
for j = 1:3
    A = A_Wind(:,:,j);
    % Tam modelde modu tanımak için özvektörün boylamsal payı kullanılır.
    % VT trim hızına bölünerek açılarla aynı ölçeğe getirilir.
    Sc = diag([1/trims{j}.row(1) ones(1, 7)]);
    [Vv, D] = eig(Sc*A/Sc);
    lam   = diag(D);
    share = sum(abs(Vv(1:4,:)).^2, 1)'./sum(abs(Vv).^2, 1)';
    [Eig_Full(:,j), Modes_Full(:,:,j), shareFull(:,j)] = sortModes(lam, share);

    lamDec = [eig(A(1:4,1:4)); eig(A(5:8,5:8))];
    [e, Modes_Dec(:,:,j)] = sortModes(lamDec, [ones(4,1); zeros(4,1)]);
    Eig_Lon(:,j) = e(1:4);
    Eig_Lat(:,j) = e(5:8);
end

for j = 1:3
    fprintf('\nProblem 3 - koşul %s, özdeğerler:\n', linName{j});
    fprintf('  tam 8x8 : %s\n', eigStr(Eig_Full(:,j)));
    fprintf('  boylamsal pay (tam): %s\n', sprintf('%.2f ', shareFull(:,j)));
    fprintf('  A11     : %s\n', eigStr(Eig_Lon(:,j)));
    fprintf('  A22     : %s\n', eigStr(Eig_Lat(:,j)));
    printModes('tam 8x8', Modes_Full(:,:,j), modeNames);
    printModes('ayrık  ', Modes_Dec(:,:,j), modeNames);
end

% s-düzlemi: üst satır bütün kökler, alt satır orijin çevresi (yavaş modlar)
fig = figure('Visible', 'off', 'Position', [100 100 1300 700]);
tl  = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact');
for row = 1:2
    for j = 1:3
        ax = nexttile(tl);  hold(ax, 'on');  grid(ax, 'on');
        plot(ax, real(Eig_Full(:,j)), imag(Eig_Full(:,j)), 'kx', 'MarkerSize', 10, 'LineWidth', 1.5);
        plot(ax, real(Eig_Lon(:,j)), imag(Eig_Lon(:,j)), 'bo', 'MarkerSize', 8, 'LineWidth', 1.2);
        plot(ax, real(Eig_Lat(:,j)), imag(Eig_Lat(:,j)), 'rs', 'MarkerSize', 8, 'LineWidth', 1.2);
        xline(ax, 0, 'k-');  yline(ax, 0, 'k-');
        if row == 1
            title(ax, sprintf('Condition %s (x_{cg} = %.2f c)', linName{j}, caseTab(j,8)));
        else
            xlim(ax, [-0.05 0.03]);  ylim(ax, [-0.15 0.15]);
            title(ax, sprintf('Condition %s, near origin', linName{j}));
        end
        xlabel(ax, 'Re [1/s]');  ylabel(ax, 'Im [rad/s]');
    end
end
legend(nexttile(tl, 1), 'full 8x8', 'longitudinal A_{11}', 'lateral A_{22}', 'Location', 'best');
exportgraphics(fig, fullfile(figDir, 'MT_Eigenvalues.png'), 'Resolution', 200);
close(fig);

%% 2. Problem 2 - Kararlılık parametreleri ve boyutsuz türevler
%  StabParams = [Xu XTu Mu Malpha Mq Lbeta Nbeta Lp Np Lr Nr]
%  StabDerivs = [CDu Cmu Cmalpha Cmq Clbeta Cnbeta Clp Cnp Clr Cnr]  (1/rad)
%
%  Rüzgâr ekseni A, durum sırası [VT alpha q theta beta p r phi]
%  (Lecture 6, s. 24 ve 100; tanımlar Lecture 5, s. 116 ve 123):
%    A(1,1) = Xu + XTu    A(3,1) = Mu + MTu    A(3,2) = Malpha + MTalpha
%    A(3,3) = Mq          (modelde alphadot türevi yok: Zalphadot = Malphadot = 0)
%  Yuvarlanma ve sapma satırları Ixz ile karışmış L', N' değerleridir:
%    [Ixx -Ixz; -Ixz Izz]*[pdot; rdot] = [L*Ixx; N*Izz]
%    -> L = L' - (Ixz/Ixx) N',   N = N' - (Ixz/Izz) L'
%  p ve r gövde ekseninde olduğundan gövde eksenindeki Ixx, Izz, Ixz kullanılır.
%
%  XTu motor modelinden bulunur: F16_6DOF/Propulsion tek başına, gaz ve irtifa
%  sabitken Mach +-dM ile koşturulur. İtki gövde x ekseni boyuncadır:
%    XTu = cos(alpha) cos(beta) (dT/dVT) / m,   Xu = A(1,1) - XTu
%  Trimdeki sürükleme ve yunuslama momenti katsayıları (Lecture 5, s. 37 ve 39):
%    CD1 = (T cos(alpha) cos(beta) - W sin(gamma)) / (qbar S)
%    Cm1 = [w x (I w + h_e)]_y / (qbar S cbar)   (dönüşte jiroskopik dengeden)
%    CDu = -Xu m VT/(qbar S) - 2 CD1,   Cmu = Mu Iyy VT/(qbar S cbar) - 2 Cm1
Ib  = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz];
hE  = [He; 0; 0];
gyro = @(w) cross(w, Ib*w + hE);            % I*wdot = M_aero - gyro(w)
StabParams = zeros(3, 11);
StabDerivs = zeros(3, 10);
P2 = struct();
for j = 1:3
    s  = trims{j}.s;  r = trims{j}.row;  A = A_Wind(:,:,j);
    VT = r(1);  al = r(2);  be = r(3);  qb = cases(j).qbar;
    Mach = caseTab(j,1);  aSnd = VT/Mach;  dM = 1e-3;

    % Uzunlamasına
    T0  = thrustF16(s.dT, Mach,      cases(j).h, thr_BP, P1_tab);
    Tp  = thrustF16(s.dT, Mach + dM, cases(j).h, thr_BP, P1_tab);
    Tm  = thrustF16(s.dT, Mach - dM, cases(j).h, thr_BP, P1_tab);
    dTdV = (Tp - Tm)/(2*dM*aSnd);
    XTu = cos(al)*cos(be)*dTdV/m;
    Xu  = A(1,1) - XTu;
    gh  = g0*(Re_SI/(Re_SI + cases(j).h))^2;     % modeldeki g(h)
    CD1 = (T0*cos(al)*cos(be) - m*gh*sin(cases(j).gamma))/(qb*S);
    CDu = -Xu*m*VT/(qb*S) - 2*CD1;
    w   = [s.p; s.q; s.r];
    gw  = gyro(w);
    Cm1 = gw(2)/(qb*S*cbar);
    Mu  = A(3,1);   Cmu = Mu*Iyy*VT/(qb*S*cbar) - 2*Cm1;
    Ma  = A(3,2);   Cma = Ma*Iyy/(qb*S*cbar);
    Mq  = A(3,3);   Cmq = 2*VT*Iyy*Mq/(qb*S*cbar^2);

    % Yanal-yönsel: satırlar [L; N], sütunlar [beta p r]
    LN = [Ixx -Ixz; -Ixz Izz]*A([6 7], [5 6 7])./[Ixx; Izz];
    Lb = LN(1,1);  Lp = LN(1,2);  Lr = LN(1,3);
    Nb = LN(2,1);  Np = LN(2,2);  Nr = LN(2,3);
    Clb = Lb*Ixx/(qb*S*b);           Cnb = Nb*Izz/(qb*S*b);
    Clp = 2*VT*Ixx*Lp/(qb*S*b^2);    Cnp = 2*VT*Izz*Np/(qb*S*b^2);
    Clr = 2*VT*Ixx*Lr/(qb*S*b^2);    Cnr = 2*VT*Izz*Nr/(qb*S*b^2);

    StabParams(j,:) = [Xu XTu Mu Ma Mq Lb Nb Lp Np Lr Nr];
    StabDerivs(j,:) = [CDu Cmu Cma Cmq Clb Cnb Clp Cnp Clr Cnr];

    % Dönüşte A'nın p, r sütunlarına eylemsizlik ve motor jiroskopik
    % terimleri de karışır. Karşılaştırma için bu terimler çıkarılmış
    % (sadece aerodinamik) değerler:
    Jg = zeros(3);
    for k = 1:3
        dw = zeros(3, 1);  dw(k) = 1e-6;
        Jg(:,k) = (gyro(w + dw) - gyro(w - dw))/(2e-6);
    end
    Mrate =Ib*[A(6,[6 3 7]); A(3,[6 3 7]); A(7,[6 3 7])] + Jg;   % sütunlar p q r
    P2(j).aeroOnly = [2*VT*Mrate(1,1)/(qb*S*b^2), 2*VT*Mrate(3,1)/(qb*S*b^2), ...
                      2*VT*Mrate(1,3)/(qb*S*b^2), 2*VT*Mrate(3,3)/(qb*S*b^2)];   % Clp Cnp Clr Cnr
    P2(j).T0 = T0;  P2(j).dTdV = dTdV;  P2(j).CD1 = CD1;  P2(j).Cm1 = Cm1;
    P2(j).CL1 = (m*gh*cos(cases(j).gamma)/cos(s.phi) - T0*sin(al))/(qb*S);
end

pNames = {'Xu', 'XTu', 'Mu', 'Malpha', 'Mq', 'Lbeta', 'Nbeta', 'Lp', 'Np', 'Lr', 'Nr'};
dNames = {'CDu', 'Cmu', 'Cmalpha', 'Cmq', 'Clbeta', 'Cnbeta', 'Clp', 'Cnp', 'Clr', 'Cnr'};
normSign = {'+/-', '+/-', '-', '-', '-', '+', '-', '+/-', '+', '-'};   % Lecture 5, s. 58 ve 96
fprintf('\nProblem 2 - trim büyüklükleri:\n%5s %9s %10s %12s %9s %9s %10s\n', 'koşul', ...
        'qbar[Pa]', 'T [N]', 'dT/dV[Ns/m]', 'CL1', 'CD1', 'Cm1');
for j = 1:3
    fprintf('%5s %9.1f %10.1f %12.3f %9.4f %9.5f %10.2e\n', linName{j}, cases(j).qbar, ...
            P2(j).T0, P2(j).dTdV, P2(j).CL1, P2(j).CD1, P2(j).Cm1);
end
fprintf('\nProblem 2 - StabParams (SI):\n%8s', '');  fprintf('%11s', linName{:});  fprintf('\n');
for k = 1:11
    fprintf('%8s', pNames{k});  fprintf('%11.4g', StabParams(:,k));  fprintf('\n');
end
fprintf('\nProblem 2 - StabDerivs (1/rad) ve normal işaret:\n%8s', '');
fprintf('%11s', linName{:});  fprintf('%9s\n', 'normal');
for k = 1:10
    fprintf('%8s', dNames{k});  fprintf('%11.4g', StabDerivs(:,k));  fprintf('%9s\n', normSign{k});
end
fprintf('\nProblem 2 - sadece aerodinamik (eylemsizlik/jiroskop terimleri çıkarılmış):\n');
fprintf('%5s %10s %10s %10s %10s\n', 'koşul', 'Clp', 'Cnp', 'Clr', 'Cnr');
for j = 1:3
    fprintf('%5s %10.4f %10.4f %10.4f %10.4f\n', linName{j}, P2(j).aeroOnly);
end
fprintf('\nStatik kararlılık: ');
for j = 1:3
    fprintf('%s: Cmalpha %s, Clbeta %s, Cnbeta %s.  ', linName{j}, ...
            ifs(StabDerivs(j,3) < 0), ifs(StabDerivs(j,5) < 0), ifs(StabDerivs(j,6) > 0));
end
fprintf('\n');
close_system('MT_thrust_harness', 0);

%% 3. Problem 1 - Trimden başlayan benzetim
%  Kumandalar trim değerinde sabit, LEF tarifeyi izler (model içinde).
%  i ve iii: 60 s; ii: psi trimden 360 deg değişene kadar.
%  Sim_* sütunları (SI, rad, dT %):
%   t VT alpha beta p q r phi theta psi xN yE h dT dh da dr
dtOut = 0.01;
Sims  = cell(3, 1);
for j = 1:3
    tr = trims{j};
    if cases(j).turn
        tEnd = 1.1*2*pi/abs(tr.d.psidot);
    else
        tEnd = 60;
    end
    Y = simFull(tr, idx, tEnd);
    t = (0:dtOut:tEnd)';
    X = interp1(Y.t, Y.X, t);
    if cases(j).turn                               % tam tur anında kes
        k = find(abs(X(:,9) - X(1,9)) >= 2*pi, 1);
        t = t(1:k);  X = X(1:k,:);
    end
    Sims{j} = [t X];
end
Sim_i = Sims{1};  Sim_ii = Sims{2};  Sim_iii = Sims{3};

fprintf('\nProblem 1 - benzetim sonunda trimden en büyük sapmalar:\n');
fprintf('%5s %7s %10s %10s %10s %10s %10s %10s %10s\n', 'koşul', 't[s]', 'dVT[m/s]', ...
        'dalpha[d]', 'dbeta[d]', 'dphi[d]', 'dtheta[d]', 'dq[d/s]', 'dh[m]');
for j = 1:3
    Z = Sims{j};  dZ = max(abs(Z - Z(1,:)), [], 1);
    fprintf('%5s %7.2f %10.3e %10.3e %10.3e %10.3e %10.3e %10.3e %10.3e\n', linName{j}, Z(end,1), ...
            dZ(2), dZ(3)*r2d, dZ(4)*r2d, dZ(8)*r2d, dZ(9)*r2d, dZ(6)*r2d, dZ(13));
end
Z  = Sim_ii;
R  = (max(Z(:,11)) - min(Z(:,11)) + max(Z(:,12)) - min(Z(:,12)))/4;   % yer izinden yarıçap
fprintf(['Koşul ii: 360 deg süresi %.2f s (trimden 2*pi/psidot = %.2f s), ' ...
         'yarıçap %.1f m (V^2/(g tan phi) = %.1f m), irtifa değişimi %.3f m\n'], ...
        Z(end,1), 2*pi/trims{2}.d.psidot, R, ...
        cases(2).VT^2/(g0*(Re_SI/(Re_SI + cases(2).h))^2*tan(cases(2).phi)), Z(end,13) - Z(1,13));

% Grafikler
yNames = {'V_T [m/s]', '\alpha [deg]', '\beta [deg]', 'p [deg/s]', 'q [deg/s]', 'r [deg/s]', ...
          '\phi [deg]', '\theta [deg]', '\psi [deg]', 'h [m]', '\delta_T [%]', ...
          '\delta_h [deg]', '\delta_a [deg]', '\delta_r [deg]'};
cols   = [2:10 13:17];
scl    = [1 r2d r2d r2d r2d r2d r2d r2d r2d 1 1 r2d r2d r2d];
for j = 1:3
    Z   = Sims{j};
    fig = figure('Visible', 'off', 'Position', [100 100 1400 800]);
    tl  = tiledlayout(fig, 4, 4, 'TileSpacing', 'compact');
    for k = 1:numel(cols)
        ax = nexttile(tl);
        plot(ax, Z(:,1), Z(:,cols(k))*scl(k), 'LineWidth', 1.2);  grid(ax, 'on');
        ylabel(ax, yNames{k});
        yl = ylim(ax);  yc = Z(1,cols(k))*scl(k);   % çok küçük değişimlerde ekseni aç
        if diff(yl) < 1e-6*max(1, abs(yc)), ylim(ax, yc + [-1 1]*1e-3*max(1, abs(yc))); end
    end
    xlabel(tl, 'Time [s]');
    title(tl, sprintf('Condition %s: trimmed-flight simulation, controls fixed at trim', linName{j}));
    exportgraphics(fig, fullfile(figDir, ['MT_Sim_' linName{j} '.png']), 'Resolution', 200);
    close(fig);
end

Z   = Sim_ii;
fig = figure('Visible', 'off', 'Position', [100 100 1200 500]);
tl  = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
ax  = nexttile(tl);
plot(ax, Z(:,12), Z(:,11), 'LineWidth', 1.5);  hold(ax, 'on');
plot(ax, Z(1,12), Z(1,11), 'go', Z(end,12), Z(end,11), 'rx', 'LineWidth', 1.5, 'MarkerSize', 9);
axis(ax, 'equal');  grid(ax, 'on');
xlabel(ax, 'y_E [m]');  ylabel(ax, 'x_N [m]');  title(ax, 'Ground track');
legend(ax, 'path', 'start', 'end (360 deg)', 'Location', 'best');
ax = nexttile(tl);
plot3(ax, Z(:,12), Z(:,11), Z(:,13), 'LineWidth', 1.5);  grid(ax, 'on');
xlabel(ax, 'y_E [m]');  ylabel(ax, 'x_N [m]');  zlabel(ax, 'h [m]');  title(ax, '3-D flight path');
title(tl, 'Condition ii: 60 deg banked level turn');
exportgraphics(fig, fullfile(figDir, 'MT_Track_ii.png'), 'Resolution', 200);
close(fig);
fprintf('Grafikler: %s\n', figDir);

%% 4. Sonuç dosyası
resFile = fullfile(here, sprintf('MT_Group%d.mat', groupNo));
save(resFile, 'Sim_i', 'Sim_ii', 'Sim_iii', 'StabParams', 'StabDerivs', ...
     'Eig_Full', 'Eig_Lon', 'Eig_Lat', 'Modes_Full', 'Modes_Dec');
fprintf('\nKaydedildi: %s\n', resFile);
whos('-file', resFile)

%% Yardımcı fonksiyonlar
function Y = simFull(tr, idx, tEnd)
% F16_6DOF'u trim noktasından başlatır, kumandalar sabit (run_HW2 ile aynı yöntem).
% Y.X sütunları: VT alpha beta p q r phi theta psi xN yE h dT dh da dr
r2d = 180/pi;  s = tr.s;
M = f16_modelInit(tr.xcg);
[~, x0] = f16_eval(M, s);
F16_6DOF([], [], [], 'term');
u = [s.dh s.da s.dr 0 0]*r2d;  u(5) = s.dT;
assignin('base', 'x0_MT', x0);
assignin('base', 'u_MT', [0 u; tEnd u]);
so = sim('F16_6DOF', 'StopTime', num2str(tEnd), 'MaxStep', '0.01', ...
         'LoadInitialState', 'on', 'InitialState', 'x0_MT', ...
         'LoadExternalInput', 'on', 'ExternalInput', 'u_MT', ...
         'SaveState', 'on', 'StateSaveName', 'xout', 'SaveFormat', 'Array');
X   = so.xout;
uvw = X(:, idx.uvw);
VT  = sqrt(sum(uvw.^2, 2));
[Y.t, iu] = unique(so.tout);
Y.X = [VT, atan2(uvw(:,3), uvw(:,1)), asin(uvw(:,2)./VT), X(:, idx.pqr), X(:, idx.euler), ...
       X(:, idx.pos(1:2)), -X(:, idx.pos(3)), repmat(s.dT, size(VT)), ...
       X(:, [idx.dh idx.da idx.dr])/r2d];
Y.X = Y.X(iu, :);
end

function str = ifs(ok)
if ok, str = 'kararlı'; else, str = 'KARARSIZ'; end
end

function T = thrustF16(thr, mach, h, thr_BP, P1_tab)
% HW-1 motor alt sistemi (F16_6DOF/Propulsion) tek başına, kararlı halde
% (P3 = P1). test_thrust_table2.m ile aynı yöntem; F16_6DOF değiştirilmez.
tm = 'MT_thrust_harness';
if ~bdIsLoaded(tm)
    new_system(tm);
    add_block('F16_6DOF/Propulsion', [tm '/Engine'], 'Position', [300 50 420 200]);
    ah = get_param([tm '/Engine'], 'PortHandles');
    vals = {'thrMT', 'machMT', 'hMT'};
    for k = 1:3
        y = 40 + 50*k;
        blk = add_block('simulink/Sources/Constant', sprintf('%s/c%d', tm, k), ...
                        'Value', vals{k}, 'Position', [120 y 200 y+20]);
        ph = get_param(blk, 'PortHandles');
        add_line(tm, ph.Outport(1), ah.Inport(k));
    end
    harnessExtraPorts(tm, ah);   % omega girişi = 0, moment çıkışı sonlandırılır
    ob  = add_block('simulink/Sinks/Out1', [tm '/y'], 'Position', [500 115 530 129]);
    oph = get_param(ob, 'PortHandles');
    add_line(tm, ah.Outport(1), oph.Inport(1));
    set_param(tm, 'StopTime', '0', 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
                  'SaveFormat', 'Array', 'ReturnWorkspaceOutputs', 'on');
end
P3old = evalin('base', 'P3_0');
assignin('base', 'thrMT', thr);
assignin('base', 'machMT', mach);
assignin('base', 'hMT', h);
assignin('base', 'P3_0', interp1(thr_BP, P1_tab, thr));
so = sim(tm);
yo = so.get('yout');
T  = yo(end, 1);
assignin('base', 'P3_0', P3old);
end
function harnessExtraPorts(tm, ah)
% Propulsion'ın itki dışındaki portları: omega_body girişi (jiroskopik moment)
% sıfır vektörüne bağlanır, moment çıkışı sonlandırılır. İtkiyi etkilemez.
for k = 4:numel(ah.Inport)
    blk = add_block('simulink/Sources/Constant', sprintf('%s/zero%d', tm, k), ...
                    'Value', 'zeros(3,1)', 'Position', [120 40+50*k 200 60+50*k]);
    ph = get_param(blk, 'PortHandles');
    add_line(tm, ph.Outport(1), ah.Inport(k));
end
for k = 2:numel(ah.Outport)
    blk = add_block('simulink/Sinks/Terminator', sprintf('%s/term%d', tm, k), ...
                    'Position', [500 115+40*k 520 135+40*k]);
    ph = get_param(blk, 'PortHandles');
    add_line(tm, ah.Outport(k), ph.Inport(1));
end
end

function str = eigStr(e)
str = strjoin(arrayfun(@(z) sprintf('%.4g%+.4gj', real(z), imag(z)), e, 'UniformOutput', false), ', ');
end

function printModes(name, T, modeNames)
fprintf('  %s %12s %8s %8s %9s %9s %9s %9s\n', name, 'mod', 'wn', 'zeta', 'periyot', ...
        'tau', 't_yarı', 't_2kat');
for k = 1:5
    fprintf('  %7s %12s %8.4f %8.4f %9.3f %9.3f %9.3f %9.3f\n', '', modeNames{k}, T(k,:));
end
end

function [e, T, sh] = sortModes(lam, share)
% Kökleri modlara ayırır. share: özvektörün boylamsal payı (0-1).
% e = [SP SP fugoid fugoid yuvarlanma spiral DR DR], T = 5x6 mod tablosu,
% sh = e'deki her kökün boylamsal payı.
%  - Dutch roll: boylamsal payı en küçük karmaşık çift
%  - kalan karmaşık çiftler: büyük |lambda| SP, küçük fugoid
%  - SP iki reel köke ayrılmışsa (tek boylamsal çift kalır), boylamsal payı
%    en büyük iki reel kök SP'dir; tabloya en sağdaki (kararsız) kök yazılır
%  - kalan iki reel kök: büyük |lambda| yuvarlanma, küçük spiral
%    (dönüşte spiral kökü boylamsal hareketle bağlaşabildiği için pay
%    yerine büyüklüğe bakılır)
isC  = abs(imag(lam)) > 1e-9*max(1, abs(lam));
cIdx = find(isC & imag(lam) > 0);          % her karmaşık çiftten bir temsilci
rIdx = find(~isC);

[~, k] = min(share(cIdx));
dr = cIdx(k);  cIdx(k) = [];
[~, o] = sort(abs(lam(cIdx)), 'descend');  cIdx = cIdx(o);

if numel(cIdx) == 2 && numel(rIdx) == 2    % olağan durum
    spI = [cIdx(1); cIdx(1)];  phI = cIdx(2);  latR = rIdx;
    spE = [lam(cIdx(1)); conj(lam(cIdx(1)))];
elseif numel(cIdx) == 1 && numel(rIdx) == 4
    [~, o] = sort(share(rIdx), 'descend');
    spR = rIdx(o(1:2));  latR = rIdx(o(3:4));
    [~, o] = sort(real(lam(spR)), 'descend');  spR = spR(o);
    spI = spR;  phI = cIdx(1);  spE = lam(spR);
else
    error('sortModes: beklenmeyen kök yapısı: %s', mat2str(lam, 4));
end
[~, o] = sort(abs(lam(latR)), 'descend');  latR = latR(o);   % yuvarlanma, spiral

ph = lam(phI);
e  = [spE; ph; conj(ph); lam(latR(1)); lam(latR(2)); lam(dr); conj(lam(dr))];
sh = share([spI; phI; phI; latR(1); latR(2); dr; dr]);
T  = [modeRow(spE(1)); modeRow(ph); modeRow(lam(latR(1))); modeRow(lam(latR(2))); modeRow(lam(dr))];
end

function r = modeRow(l)
% [wn zeta periyot tau t_yarı t_ikikat]
r = nan(1, 6);
s = real(l);  w = abs(imag(l));
if w > 0
    r(1) = abs(l);  r(2) = -s/abs(l);  r(3) = 2*pi/w;
end
r(4) = -1/s;
if s < 0, r(5) = log(2)*r(4); else, r(6) = -log(2)*r(4); end
end
