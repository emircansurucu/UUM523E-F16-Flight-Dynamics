%% run_MT.m
%  UUM 523E - Midterm Project (Grup 2). F-16 trimde uçuş, kararlılık
%  parametreleri ve dinamik modlar.
%
%   0. Hazırlık: koşul i, ii, iii için trim ve rüzgâr ekseni doğrusal modeli
%      (HW-2 ile aynı fonksiyonlar)
%        A_sch: dlef tarifeyi izler   -> Problem 3 (HW2_Group2.mat ile aynı)
%        A_fix: dlef trimde sabit     -> Problem 2 (ödevde istendiği gibi)
%   1. Problem 1: trimden başlayan doğrusal olmayan benzetim -> Sim_i, Sim_ii, Sim_iii
%   2. Problem 2: kararlılık parametreleri ve boyutsuz türevler -> StabParams, StabDerivs
%   3. Problem 3: tam 8x8 ve ayrık (A11, A22) modellerin modları
%                 -> Eig_Full, Eig_Lon, Eig_Lat, Modes_Full, Modes_Dec
%   4. Sonuç dosyası: MT_Group2.mat (sadece bu 10 değişken)
%
%  Koşullar (HW-2 Tablo 1): i = durum 1 (düz uçuş, xcg 0.25c),
%  ii = durum 4 (60 deg yatışlı dönüş), iii = durum 6 (düz uçuş, xcg 0.35c).
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
    Tatm = T0_SI*Tfac;
    V    = caseTab(j,1)*sqrt(gamma_R_SI*Tatm);
    cases(j).VT    = V;
    cases(j).h     = h;
    cases(j).qps   = 0.5*V^2/(R_ps_SI*Tatm);
    cases(j).gamma = caseTab(j,3)*d2r;
    cases(j).beta  = caseTab(j,4)*d2r;
    cases(j).phi   = caseTab(j,5)*d2r;
    cases(j).nz    = caseTab(j,6);
    cases(j).turn  = caseTab(j,7) == 1;
    cases(j).qbar  = 0.5*R0_SI*Tfac^4.14*V^2;
end

trims = cell(3, 1);
A_sch = zeros(8, 8, 3);  A_fix = zeros(8, 8, 3);
for xc = [0.25 0.35]
    M = f16_modelInit(xc);
    for j = find(caseTab(:,8) == xc)'
        trims{j} = trim_F16(M, cases(j));
        A_sch(:,:,j) = linearize_F16(M, trims{j}, 'wind', 'tarife');
        A_fix(:,:,j) = linearize_F16(M, trims{j}, 'wind', 'sabit');
    end
    F16_6DOF([], [], [], 'term');
end

hw2 = load(fullfile(here, 'HW2_Group2.mat'), 'A_Wind');
fprintf('\nHazırlık - trim (deg) ve HW2_Group2.mat ile fark:\n');
fprintf('%5s %9s %8s %8s %8s %8s %8s %8s %9s %10s\n', 'koşul', 'VT[m/s]', 'alpha', ...
        'phi', 'theta', 'dh', 'dlef', 'dT[%]', 'J', 'A farkı');
for j = 1:3
    r  = trims{j}.row;
    eA = norm(A_sch(:,:,j) - hw2.A_Wind(:,:,j))/norm(hw2.A_Wind(:,:,j));
    fprintf('%5s %9.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %9.2e %10.2e\n', linName{j}, ...
            r(1), r([2 4 5 11 14])*r2d, r(10), r(15), eA);
end

%% 1. Problem 1 - Trimden başlayan benzetim
%  Kumandalar trim değerinde sabit, dlef tarifeyi izler (model içinde).
%  i ve iii: 60 s; ii: psi 360 deg değişene kadar.
%  Sim_* sütunları: t VT alpha beta p q r phi theta psi xN yE h dT dh da dr
Sims = cell(3, 1);
for j = 1:3
    tr = trims{j};
    tEnd = 60;
    if cases(j).turn, tEnd = 1.1*2*pi/tr.d.psidot; end
    Z = sim_F16(tr, tEnd);
    t = (0:0.01:tEnd)';
    Z = [t, interp1(Z(:,1), Z(:,2:end), t)];
    if cases(j).turn                               % tam tur anında kes
        k = find(abs(Z(:,10) - Z(1,10)) >= 2*pi, 1);
        Z = Z(1:k,:);
    end
    Sims{j} = Z;
end
Sim_i = Sims{1};  Sim_ii = Sims{2};  Sim_iii = Sims{3};

fprintf('\nProblem 1 - benzetimde trimden en büyük sapmalar:\n');
fprintf('%5s %7s %10s %10s %10s %10s %10s %10s %10s\n', 'koşul', 't[s]', 'dVT[m/s]', ...
        'dalpha[d]', 'dbeta[d]', 'dphi[d]', 'dtheta[d]', 'dq[d/s]', 'dh[m]');
for j = 1:3
    Z = Sims{j};  dZ = max(abs(Z - Z(1,:)), [], 1);
    fprintf('%5s %7.2f %10.3e %10.3e %10.3e %10.3e %10.3e %10.3e %10.3e\n', linName{j}, Z(end,1), ...
            dZ(2), dZ(3)*r2d, dZ(4)*r2d, dZ(8)*r2d, dZ(9)*r2d, dZ(6)*r2d, dZ(13));
end
Z = Sim_iii;
fprintf('Koşul iii: theta sapması 20/40/60 s: %.2e / %.2e / %.2e deg\n', ...
        abs(Z([2001 4001 6001],9) - Z(1,9))*r2d);
Z = Sim_ii;
R = (max(Z(:,11)) - min(Z(:,11)) + max(Z(:,12)) - min(Z(:,12)))/4;   % yer izinden yarıçap
gh = g0*(Re_SI/(Re_SI + cases(2).h))^2;                               % modeldeki g(h)
fprintf(['Koşul ii: 360 deg süresi %.2f s (2*pi/psidot = %.2f s), yarıçap %.1f m ' ...
         '(V^2/(g tan phi) = %.1f m), irtifa değişimi %.3f m\n'], Z(end,1), ...
        2*pi/trims{2}.d.psidot, R, cases(2).VT^2/(gh*tan(cases(2).phi)), Z(end,13) - Z(1,13));

% Grafikler
yNames = {'V_T [m/s]', '\alpha [deg]', '\beta [deg]', 'p [deg/s]', 'q [deg/s]', 'r [deg/s]', ...
          '\phi [deg]', '\theta [deg]', '\psi [deg]', 'h [m]', '\delta_T [%]', ...
          '\delta_h [deg]', '\delta_a [deg]', '\delta_r [deg]'};
cols = [2:10 13:17];
scl  = [1 r2d r2d r2d r2d r2d r2d r2d r2d 1 1 r2d r2d r2d];
for j = 1:3
    Z   = Sims{j};
    fig = figure('Visible', 'off', 'Position', [100 100 1400 800]);
    tl  = tiledlayout(fig, 4, 4, 'TileSpacing', 'compact');
    for k = 1:numel(cols)
        ax = nexttile(tl);
        plot(ax, Z(:,1), Z(:,cols(k))*scl(k), 'LineWidth', 1.2);  grid(ax, 'on');
        ylabel(ax, yNames{k});
        yl = ylim(ax);  yc = Z(1,cols(k))*scl(k);   % sabit kalan sinyallerde ekseni aç
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

%% 2. Problem 2 - Kararlılık parametreleri ve boyutsuz türevler (A_fix)
%  StabParams = [Xu XTu Mu Malpha Mq Lbeta Nbeta Lp Np Lr Nr]
%  StabDerivs = [CDu Cmu Cmalpha Cmq Clbeta Cnbeta Clp Cnp Clr Cnr]  (1/rad)
%
%  Rüzgâr ekseni A, durum sırası [VT alpha q theta beta p r phi]
%  (Lecture 6, s. 24 ve 100; tanımlar Lecture 5, s. 116 ve 123):
%    A(1,1) = Xu + XTu    A(3,1) = Mu + MTu    A(3,2) = Malpha + MTalpha
%    A(3,3) = Mq          (modelde alphadot türevi yok: Zalphadot = Malphadot = 0)
%  Yuvarlanma ve sapma satırları Ixz ile karışmış L', N' değerleridir:
%    [Ixx -Ixz; -Ixz Izz]*[pdot; rdot] = [L*Ixx; N*Izz]
%  İtki CG'den geçtiği için MTu = MTalpha = 0.
%
%  XTu motor modelinden bulunur: F16_6DOF/Propulsion tek başına, gaz ve irtifa
%  sabitken Mach +-dM ile koşturulur. İtki gövde x ekseni boyuncadır:
%    XTu = cos(alpha) cos(beta) (dT/dVT) / m,   Xu = A(1,1) - XTu
%  Trimdeki sürükleme katsayısı (Lecture 5, s. 37):
%    CD1 = (T cos(alpha) cos(beta) - W sin(gamma)) / (qbar S)
%    CDu = -Xu m VT/(qbar S) - 2 CD1,   Cmu = Mu Iyy VT/(qbar S cbar)   (trimde Cm1 = 0)
StabParams = zeros(3, 11);
StabDerivs = zeros(3, 10);
aeroOnly   = zeros(3, 4);
P2trim     = zeros(3, 5);       % T1, dT/dV, CL1, CD1, qbar
Ib = [Ixx 0 -Ixz; 0 Iyy 0; -Ixz 0 Izz];
for j = 1:3
    s  = trims{j}.s;  r = trims{j}.row;  A = A_fix(:,:,j);
    VT = r(1);  al = r(2);  be = r(3);  qb = cases(j).qbar;
    Mach = caseTab(j,1);  dM = 1e-3;

    % Uzunlamasına
    T0   = thrustF16(s.dT, Mach, cases(j).h);
    dTdV = (thrustF16(s.dT, Mach + dM, cases(j).h) - thrustF16(s.dT, Mach - dM, cases(j).h)) ...
           /(2*dM*VT/Mach);
    XTu = cos(al)*cos(be)*dTdV/m;
    Xu  = A(1,1) - XTu;
    gh  = g0*(Re_SI/(Re_SI + cases(j).h))^2;
    CD1 = (T0*cos(al)*cos(be) - m*gh*sin(cases(j).gamma))/(qb*S);
    CL1 = (m*gh*cos(cases(j).gamma)/cos(s.phi) - T0*sin(al))/(qb*S);
    CDu = -Xu*m*VT/(qb*S) - 2*CD1;
    Mu  = A(3,1);   Cmu = Mu*Iyy*VT/(qb*S*cbar);
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
    P2trim(j,:)     = [T0 dTdV CL1 CD1 qb];

    % Dönüşte A'nın p, r sütunlarına w x (I w + hE) teriminin türevi de girer.
    % Bu terim eklenerek sadece aerodinamik sönüm türevleri bulunur:
    %   d/dw [w x H] = [w]x I - [H]x,   H = I w + hE
    w  = [s.p; s.q; s.r];
    H  = Ib*w + [He; 0; 0];
    Jg = capraz(w)*Ib - capraz(H);
    Mrate = Ib*A([6 3 7], [6 7]) + Jg(:, [1 3]);          % satırlar L M N, sütunlar p r
    aeroOnly(j,:) = 2*VT/(qb*S*b^2)*[Mrate(1,1) Mrate(3,1) Mrate(1,2) Mrate(3,2)];   % Clp Cnp Clr Cnr
end
close_system('thrust_harness', 0);

pNames = {'Xu', 'XTu', 'Mu', 'Malpha', 'Mq', 'Lbeta', 'Nbeta', 'Lp', 'Np', 'Lr', 'Nr'};
dNames = {'CDu', 'Cmu', 'Cmalpha', 'Cmq', 'Clbeta', 'Cnbeta', 'Clp', 'Cnp', 'Clr', 'Cnr'};
normSign = {'+/-', '+/-', '-', '-', '-', '+', '-', '+/-', '+', '-'};   % Lecture 5, s. 58 ve 96
fprintf('\nProblem 2 - trim büyüklükleri:\n%5s %10s %12s %9s %9s %9s\n', 'koşul', ...
        'T [N]', 'dT/dV[Ns/m]', 'CL1', 'CD1', 'qbar[Pa]');
for j = 1:3
    fprintf('%5s %10.1f %12.3f %9.4f %9.5f %9.1f\n', linName{j}, P2trim(j,:));
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
fprintf('\nProblem 2 - sadece aerodinamik sönüm türevleri:\n%5s %10s %10s %10s %10s\n', ...
        'koşul', 'Clp', 'Cnp', 'Clr', 'Cnr');
for j = 1:3
    fprintf('%5s %10.4f %10.4f %10.4f %10.4f\n', linName{j}, aeroOnly(j,:));
end

%% 3. Problem 3 - Dinamik modlar (A_sch)
%  Modes_* satırları: SP, fugoid, yuvarlanma, spiral, Dutch roll
%  Sütunlar: wn [rad/s], zeta, periyot [s], tau [s], t_yarı [s], t_iki kat [s]
%  Uygulanmayan değerler NaN. tau = -1/Re(lambda) (Lecture 6); kararsız kökte tau < 0.
%  Tam modelde kökler özvektörün boylamsal payına göre ayrılır: payı en büyük
%  dört kök boylamsal kabul edilir (VT, trim hızına bölünerek açılarla aynı ölçeğe getirildi).
Eig_Full   = complex(zeros(8, 3));
Eig_Lon    = complex(zeros(4, 3));
Eig_Lat    = complex(zeros(4, 3));
Modes_Full = nan(5, 6, 3);
Modes_Dec  = nan(5, 6, 3);
modeNames  = {'SP', 'fugoid', 'yuvarlanma', 'spiral', 'Dutch roll'};
for j = 1:3
    A  = A_sch(:,:,j);
    Sc = diag([1/trims{j}.row(1) ones(1, 7)]);
    [V, D] = eig(Sc*A/Sc);
    lam   = diag(D);
    share = sum(abs(V(1:4,:)).^2, 1)'./sum(abs(V).^2, 1)';
    [~, o] = sort(share, 'descend');
    Eig_Full(:,j) = lam(o);
    Eig_Lon(:,j)  = eig(A(1:4,1:4));
    Eig_Lat(:,j)  = eig(A(5:8,5:8));
    Modes_Full(:,:,j) = modTablosu(lam(o(1:4)), lam(o(5:8)));
    Modes_Dec(:,:,j)  = modTablosu(Eig_Lon(:,j), Eig_Lat(:,j));

    fprintf('\nProblem 3 - koşul %s:\n', linName{j});
    fprintf('  tam 8x8 (boylamsal paya göre sıralı): %s\n', eigStr(lam(o)));
    fprintf('  boylamsal pay: %s\n', sprintf('%.2f ', share(o)));
    fprintf('  A11: %s\n  A22: %s\n', eigStr(Eig_Lon(:,j)), eigStr(Eig_Lat(:,j)));
    fprintf('  %-12s %8s %8s %9s %9s %9s %9s   |  ayrık: %8s %8s %9s %9s\n', 'mod', 'wn', 'zeta', ...
            'periyot', 'tau', 't_yarı', 't_2kat', 'wn', 'zeta', 'periyot', 'tau');
    for k = 1:5
        fprintf('  %-12s %8.4f %8.4f %9.3f %9.3f %9.3f %9.3f   |  %14.4f %8.4f %9.3f %9.3f\n', ...
                modeNames{k}, Modes_Full(k,:,j), Modes_Dec(k,1:4,j));
    end
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
            xlim(ax, [-0.1 0.05]);  ylim(ax, [-0.15 0.15]);
            title(ax, sprintf('Condition %s, near origin', linName{j}));
        end
        xlabel(ax, 'Re [1/s]');  ylabel(ax, 'Im [rad/s]');
    end
end
legend(nexttile(tl, 1), 'full 8x8', 'longitudinal A_{11}', 'lateral A_{22}', 'Location', 'best');
exportgraphics(fig, fullfile(figDir, 'MT_Eigenvalues.png'), 'Resolution', 200);
close(fig);

% Karşılaştırma için dlef sabitken (A_fix) özdeğerler
fprintf('\nLEF sabitken (A_fix) tam modelin özdeğerleri:\n');
for j = 1:3, fprintf('  %-4s %s\n', linName{j}, eigStr(eig(A_fix(:,:,j)))); end

%% 4. Sonuç dosyası
resFile = fullfile(here, sprintf('MT_Group%d.mat', groupNo));
save(resFile, 'Sim_i', 'Sim_ii', 'Sim_iii', 'StabParams', 'StabDerivs', ...
     'Eig_Full', 'Eig_Lon', 'Eig_Lat', 'Modes_Full', 'Modes_Dec');
fprintf('\nKaydedildi: %s\n', resFile);
whos('-file', resFile)

%% Yardımcı fonksiyonlar
function T = modTablosu(lon, lat)
% lon: 4 boylamsal kök, lat: 4 yanal kök -> 5x6 mod tablosu
%  SP: büyüklüğü en büyük boylamsal kök (reel ise o kök, karmaşık ise çifti)
%  fugoid: kalan karmaşık çift
%  Dutch roll: yanal karmaşık çift; yuvarlanma ve spiral: büyük ve küçük reel kök
[~, o] = sort(abs(lon), 'descend');  lon = lon(o);
sp = lon(1);
kalan = lon(abs(lon - sp) > 1e-9 & abs(lon - conj(sp)) > 1e-9);
fug = kalan(imag(kalan) > 0);
if isempty(fug), fug = NaN; end
dr = lat(imag(lat) > 0);
re = sort(lat(imag(lat) == 0), 'descend', 'ComparisonMethod', 'abs');
T = [modSatiri(sp); modSatiri(fug(1)); modSatiri(re(1)); modSatiri(re(2)); modSatiri(dr)];
end

function r = modSatiri(l)
% [wn zeta periyot tau t_yarı t_ikikat]
r = nan(1, 6);
s = real(l);  w = abs(imag(l));
if w > 0
    r(1) = abs(l);  r(2) = -s/abs(l);  r(3) = 2*pi/w;
end
r(4) = -1/s;
if s < 0, r(5) = log(2)*r(4); else, r(6) = -log(2)*r(4); end
end

function X = capraz(a)
% a x b = capraz(a)*b
X = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
end

function str = eigStr(e)
str = strjoin(arrayfun(@(z) sprintf('%.4g%+.4gj', real(z), imag(z)), e, 'UniformOutput', false), ', ');
end

function T = thrustF16(thr, mach, h)
% HW-1 motor alt sistemi (F16_6DOF/Propulsion) tek başına, kararlı halde
% (P3 = P1). test_thrust_table2.m ile aynı test modeli; F16_6DOF değiştirilmez.
tm = 'thrust_harness';
if ~bdIsLoaded(tm)
    new_system(tm);
    add_block('F16_6DOF/Propulsion', [tm '/Engine'], 'Position', [300 50 420 200]);
    ah = get_param([tm '/Engine'], 'PortHandles');
    giris = {'thrMT', 'machMT', 'hMT', 'zeros(3,1)'};     % 4. giriş omega (itkiyi etkilemez)
    for k = 1:numel(ah.Inport)
        blk = add_block('simulink/Sources/Constant', sprintf('%s/c%d', tm, k), ...
                        'Value', giris{k}, 'Position', [120 40+50*k 200 60+50*k]);
        ph = get_param(blk, 'PortHandles');
        add_line(tm, ph.Outport(1), ah.Inport(k));
    end
    blk = add_block('simulink/Sinks/Out1', [tm '/T'], 'Position', [500 115 530 129]);
    ph  = get_param(blk, 'PortHandles');
    add_line(tm, ah.Outport(1), ph.Inport(1));
    for k = 2:numel(ah.Outport)                          % moment çıkışı kullanılmıyor
        blk = add_block('simulink/Sinks/Terminator', sprintf('%s/t%d', tm, k), ...
                        'Position', [500 115+40*k 520 135+40*k]);
        ph = get_param(blk, 'PortHandles');
        add_line(tm, ah.Outport(k), ph.Inport(1));
    end
    set_param(tm, 'StopTime', '0', 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
                  'SaveFormat', 'Array', 'ReturnWorkspaceOutputs', 'on');
end
P3eski = evalin('base', 'P3_0');
assignin('base', 'thrMT', thr);
assignin('base', 'machMT', mach);
assignin('base', 'hMT', h);
assignin('base', 'P3_0', interp1(evalin('base', 'thr_BP'), evalin('base', 'P1_tab'), thr));
so = sim(tm);
y  = so.get('yout');
T  = y(end, 1);
assignin('base', 'P3_0', P3eski);
end
