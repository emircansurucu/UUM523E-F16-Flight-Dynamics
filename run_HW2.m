%% run_HW2.m
%  UUM 523E - HW-2 (Grup 2). F-16 trim ve doğrusallaştırma.
%
%   1. Problem 1: Tablo 1'deki 6 trim durumu (fmincon)        -> TrimResults (6x15)
%   2. Problem 2a: gövde ekseninde doğrusal model (i, ii, iii)  -> A_Body, B_Body
%   3. Problem 2b: rüzgâr ekseninde doğrusal model (i, ii, iii) -> A_Wind, B_Wind
%   4. Sonuç dosyası: HW2_Group2.mat (sadece bu 5 değişken)
%   5. Doğrulama: trim artıkları, gövde/rüzgâr tutarlılığı, trimden başlayan
%      benzetim, doğrusal ve doğrusal olmayan yanıtın karşılaştırılması
%
%  Bütün sonuçlar HW-1 modelinden (F16_6DOF.slx) gelir. Model API'si ile
%  çağrılır ve değiştirilmez; sadece xcg değişkeni güncellenir.
%
%  Çalıştırma:  >> run_HW2

here = fileparts(mfilename('fullpath'));
run(fullfile(here, 'init_F16.m'));
groupNo = 2;
figDir  = fullfile(here, 'figures');
load_system(fullfile(here, 'F16_6DOF.slx'));
% Model API'sine durum vektörü dizi olarak verildiği için çıkan uyarıyı kapat
warning('off', 'Simulink:Engine:InitStatesInArrayFormatShouldBeAvoided');
d2r = pi/180;  r2d = 180/pi;

%% 1. Problem 1 - Trim durumları (Tablo 1)
%  NaN: belirtilmemiş, trimde bulunur.
%          Mach  h [ft]  gamma  beta  phi  nz   dönüş  xcg
caseTab = [ 0.6  10000    0     NaN    0   NaN   0     0.25;
            0.3   3000   -3     NaN    0   NaN   0     0.25;
            0.5   3000   10     NaN    0   NaN   0     0.25;
            0.6  10000    0      0    60   NaN   1     0.25;
            0.6  10000    0      0   NaN    4    1     0.25;
            0.6  10000    0     NaN    0   NaN   0     0.35 ];
nCase = size(caseTab, 1);

% Uçuş koşulu: Mach ve irtifadan VT ve qbar/p_s (Stevens-Lewis-Johnson App. A.6, SI)
for k = 1:nCase
    h    = caseTab(k,2)*ft2m;
    Tfac = 1 - lapse_SI*h;
    Tatm = T0_SI*Tfac;   if h >= h_trop_SI, Tatm = T_iso_SI; end
    V    = caseTab(k,1)*sqrt(gamma_R_SI*Tatm);
    cases(k).VT    = V;
    cases(k).h     = h;
    cases(k).qps   = 0.5*V^2/(R_ps_SI*Tatm);       % qbar/p_s = (rho V^2/2)/(R rho T)
    cases(k).gamma = caseTab(k,3)*d2r;
    cases(k).beta  = caseTab(k,4)*d2r;
    cases(k).phi   = caseTab(k,5)*d2r;
    cases(k).nz    = caseTab(k,6);
    cases(k).turn  = caseTab(k,7) == 1;
    cases(k).xcg   = caseTab(k,8);
end

% Doğrusallaştırma noktaları: i = durum 1, ii = durum 4, iii = durum 6
linCase = [1 4 6];
linName = {'i', 'ii', 'iii'};

TrimResults = zeros(nCase, 15);
A_Body = zeros(8, 8, 3);  B_Body = zeros(8, 4, 3);
A_Wind = zeros(8, 8, 3);  B_Wind = zeros(8, 4, 3);
trims  = cell(nCase, 1);
for xc = [0.25 0.35]
    M = f16_modelInit(xc);
    for k = find(caseTab(:,8) == xc)'
        trims{k} = trim_F16(M, cases(k));
        TrimResults(k,:) = trims{k}.row;
    end
    for j = find(caseTab(linCase,8) == xc)'
        tr = trims{linCase(j)};
        [A_Body(:,:,j), B_Body(:,:,j)] = linearize_F16(M, tr, 'body');
        [A_Wind(:,:,j), B_Wind(:,:,j)] = linearize_F16(M, tr, 'wind');
    end
    F16_6DOF([], [], [], 'term');
end
idx = M.idx;

colNames = {'VT','alpha','beta','phi','theta','psi/psid','p','q','r','dT[%]','dh','da','dr','dlef','J'};
fprintf('\nProblem 1 - TrimResults (SI, rad, rad/s, dT %%):\n%5s', 'durum');
fprintf('%12s', colNames{:});  fprintf('\n');
for k = 1:nCase, fprintf('%5d', k);  fprintf('%12.5g', TrimResults(k,:));  fprintf('\n'); end

fprintf('\nTrim özeti (deg):\n%5s %8s %8s %8s %8s %8s %8s %8s %8s %8s %6s %5s\n', 'durum', ...
        'alpha', 'beta', 'phi', 'theta', 'dh', 'da', 'dr', 'dlef', 'dT[%]', 'nz', 'exit');
for k = 1:nCase
    fprintf('%5d %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %6.3f %5d\n', k, ...
            TrimResults(k,[2 3 4 5 11 12 13 14])*r2d, TrimResults(k,10), trims{k}.d.nz, trims{k}.exitflag);
end

%% 2-3. Problem 2 - Doğrusal modeller
xB = {'u','w','q','theta','v','p','r','phi'};
xW = {'VT','alpha','q','theta','beta','p','r','phi'};
uN = {'dT','dh','da','dr'};
for j = 1:3
    printMat(['A_Body (' linName{j} ')'], A_Body(:,:,j), xB, xB);
    printMat(['B_Body (' linName{j} ')'], B_Body(:,:,j), xB, uN);
end
for j = 1:3
    printMat(['A_Wind (' linName{j} ')'], A_Wind(:,:,j), xW, xW);
    printMat(['B_Wind (' linName{j} ')'], B_Wind(:,:,j), xW, uN);
end

fprintf('\nÖzdeğerler (gövde ekseni A):\n');
for j = 1:3
    fprintf('  %-4s boylamsal (A11): %s\n', linName{j}, eigStr(A_Body(1:4,1:4,j)));
    fprintf('       yanal (A22):     %s\n', eigStr(A_Body(5:8,5:8,j)));
    fprintf('       tam 8x8:         %s\n', eigStr(A_Body(:,:,j)));
end

%% 4. Sonuç dosyası
resFile = fullfile(here, sprintf('HW2_Group%d.mat', groupNo));
save(resFile, 'TrimResults', 'A_Body', 'B_Body', 'A_Wind', 'B_Wind');
fprintf('\nKaydedildi: %s\n', resFile);

%% 5. Doğrulama
% 5a. Trimde modelin türevleri (konum ve psi hariç) ve kısıtlar sıfır olmalı
fprintf('\nDoğrulama 5a - trimdeki artıklar (en büyük mutlak değer):\n');
fprintf('%5s %12s %12s %12s %12s\n', 'durum', 'uvw [m/s2]', 'pqr [r/s2]', 'eyl/motor', 'kısıt');
for k = 1:nCase
    d = trims{k}.d;
    fprintf('%5d %12.2e %12.2e %12.2e %12.2e\n', k, max(abs([d.udot d.vdot d.wdot])), ...
            max(abs([d.pdot d.qdot d.rdot d.phidot d.thetadot])), max(abs(d.aux)), max(abs(trims{k}.ceq)));
end

% 5b. İki model aynı sistemi anlatmalı: A_W = T*A_B*inv(T), B_W = T*B_B
fprintf('\nDoğrulama 5b - gövde/rüzgâr tutarlılığı (bağıl fark):\n');
for j = 1:3
    s = trims{linCase(j)}.s;
    T = bodyToWind(s.u, s.v, s.w);
    eA = norm(T*A_Body(:,:,j)/T - A_Wind(:,:,j))/norm(A_Wind(:,:,j));
    eB = norm(T*B_Body(:,:,j) - B_Wind(:,:,j))/norm(B_Wind(:,:,j));
    fprintf('  %-4s  A: %.2e   B: %.2e\n', linName{j}, eA, eB);
end

% 5c. Trimden başlayan tam model benzetimi (10 s, kumandalar sabit).
%     Tırmanış ve alçalışta irtifa ile yoğunluk değiştiği için uzun sürede VT kayar;
%     bu yüzden gamma ilk 1 s'den ölçülür.
fprintf('\nDoğrulama 5c - trimden başlayan tam model benzetimi (10 s):\n');
fprintf('%5s %10s %10s %10s %10s %10s %11s %11s %11s %11s\n', 'durum', 'dVT[m/s]', 'dalpha[d]', ...
        'dbeta[d]', 'dphi[d]', 'dq[d/s]', 'gam_sim[d]', 'gam_trim[d]', 'psid_sim', 'psid_trim');
for k = 1:nCase
    s = trims{k}.s;
    u = [s.dh s.da s.dr 0 0]*r2d;  u(5) = s.dT;
    Y = simFull(trims{k}, idx, 10, [0 u; 10 u]);
    i1   = find(Y.t <= 1, 1, 'last');
    gSim = asin((Y.h(i1) - Y.h(1))/Y.t(i1)/Y.VT(1))*r2d;
    pSim = (Y.psi(end) - Y.psi(1))/Y.t(end);
    fprintf('%5d %10.2e %10.2e %10.2e %10.2e %10.2e %11.5f %11.5f %11.6f %11.6f\n', k, ...
            max(abs(Y.VT - Y.VT(1))), max(abs(Y.alpha - Y.alpha(1)))*r2d, ...
            max(abs(Y.beta - Y.beta(1)))*r2d, max(abs(Y.phi - Y.phi(1)))*r2d, ...
            max(abs(Y.q - Y.q(1)))*r2d, gSim, cases(k).gamma*r2d, pSim, trims{k}.d.psidot);
end

% 5d. Doğrusal ve doğrusal olmayan yanıt (koşul i ve iii)
%     Üst satır: stabilatör ikilisi (1-2 s +, 2-3 s -), alt satır: kanatçık darbesi (1-2 s).
%     Koşul iii boylamsal olarak kararsız (xcg = 0.35c), genlik küçük ve süre kısa tutuldu.
cmpAmp = [1 0 0.1];  cmpT = [10 0 5];      % deg, s
for j = [1 3]
    tr  = trims{linCase(j)};
    fig = respCompare(tr, idx, A_Wind(:,:,j), B_Wind(:,:,j), cmpT(j), cmpAmp(j));
    title(fig.Children(1), sprintf(['Condition %s (x_{cg} = %.2f c): %.1f deg \\delta_h doublet (top), ' ...
          '%.1f deg \\delta_a pulse (bottom)'], linName{j}, tr.xcg, cmpAmp(j), cmpAmp(j)));
    exportgraphics(fig, fullfile(figDir, ['HW2_LinVsNonlin_' linName{j} '.png']), 'Resolution', 200);
end
fprintf('\nGrafikler: %s\n', figDir);

% 5e. LEF tarifesinin etkisi: tam modelin özdeğerleri (eyleyici, motor ve LEF
%     durumları dahil; x, y ve psi hariç 15 durum), LEF açık ve dondurulmuş.
fprintf('\nDoğrulama 5e - tam modelin özdeğerleri (koşul i ve iii):\n');
for j = [1 3]
    tr = trims{linCase(j)};
    M  = f16_modelInit(tr.xcg);
    [~, x0, u0] = f16_eval(M, tr.s);
    keep = setdiff(1:M.nx, [idx.pos(1:2); idx.euler(3)]);
    n  = numel(keep);
    Af = zeros(n);
    for k = 1:n
        dx = zeros(M.nx, 1);  dx(keep(k)) = 1e-4*max(1, abs(x0(keep(k))));
        F16_6DOF(0, x0 + dx, u0, 'outputs');  fp = F16_6DOF(0, x0 + dx, u0, 'derivs');
        F16_6DOF(0, x0 - dx, u0, 'outputs');  fm = F16_6DOF(0, x0 - dx, u0, 'derivs');
        Af(:,k) = (fp(keep) - fm(keep))/(2*dx(keep(k)));
    end
    F16_6DOF([], [], [], 'term');
    noLef = ~ismember(keep, [idx.lefTF idx.dlef]);
    fprintf('  %-4s LEF açık:       %s\n', linName{j}, eigStr(Af));
    fprintf('       LEF dondurulmuş: %s\n', eigStr(Af(noLef, noLef)));
end

%% Yardımcı fonksiyonlar
function printMat(name, X, rowN, colN)
fprintf('\n%s:\n%8s', name, '');  fprintf('%12s', colN{:});  fprintf('\n');
for i = 1:size(X,1), fprintf('%8s', rowN{i});  fprintf('%12.4g', X(i,:));  fprintf('\n'); end
end

function str = eigStr(A)
e = sort(eig(A));
str = strjoin(arrayfun(@(z) sprintf('%.4g%+.4gj', real(z), imag(z)), e, 'UniformOutput', false), ', ');
end

function T = bodyToWind(u, v, w)
% [VT alpha beta]'nın [u v w]'ye göre Jacobian'ı, 8 durumlu sıraya yerleştirilmiş.
% Gövde sırası [u w q theta v p r phi], rüzgâr sırası [VT alpha q theta beta p r phi].
VT  = sqrt(u^2 + v^2 + w^2);
uw2 = u^2 + w^2;
J = [ u/VT,                        v/VT,              w/VT;
     -w/uw2,                       0,                 u/uw2;
     -u*v/(VT^2*sqrt(uw2)),        sqrt(uw2)/VT^2,   -w*v/(VT^2*sqrt(uw2))];
T = eye(8);
T([1 2 5], :) = 0;
T([1 2 5], [1 5 2]) = J;     % satırlar VT, alpha, beta; sütunlar u, v, w
end

function Y = simFull(tr, idx, tEnd, uExt)
% F16_6DOF'u trim noktasından başlatır. uExt = [t, dh da dr dsb throttle] (deg, %).
% Durumlar Array formatında kaydedilir; sıra model API'sindekiyle aynı.
M = f16_modelInit(tr.xcg);
[~, x0] = f16_eval(M, tr.s);
F16_6DOF([], [], [], 'term');
assignin('base', 'x0_HW2', x0);
assignin('base', 'u_HW2', uExt);
so = sim('F16_6DOF', 'StopTime', num2str(tEnd), 'MaxStep', '0.01', ...
         'LoadInitialState', 'on', 'InitialState', 'x0_HW2', ...
         'LoadExternalInput', 'on', 'ExternalInput', 'u_HW2', ...
         'SaveState', 'on', 'StateSaveName', 'xout', 'SaveFormat', 'Array');
X = so.xout;
Y.t     = so.tout;
uvw     = X(:, idx.uvw);
Y.VT    = sqrt(sum(uvw.^2, 2));
Y.alpha = atan2(uvw(:,3), uvw(:,1));
Y.beta  = asin(uvw(:,2)./Y.VT);
Y.p     = X(:, idx.pqr(1));
Y.q     = X(:, idx.pqr(2));
Y.r     = X(:, idx.pqr(3));
Y.phi   = X(:, idx.euler(1));
Y.theta = X(:, idx.euler(2));
Y.psi   = X(:, idx.euler(3));
Y.h     = -X(:, idx.pos(3));
end

function fig = respCompare(tr, idx, A, B, tEnd, amp)
% Doğrusal model (rüzgâr ekseni) sıfır dereceli tutucu ile ayrıklaştırılır,
% tam model aynı girişlerle koşturulur. Yanıtlar trimden sapma olarak çizilir.
s   = tr.s;  r2d = 180/pi;
dt  = 0.01;  t = (0:dt:tEnd)';
dhP = amp*((t >= 1 & t < 2) - (t >= 2 & t < 3));   % deg
daP = amp*(t >= 1 & t < 2);                         % deg
names = {'\Delta V_T [m/s]', '\Delta\alpha [deg]', '\Delta q [deg/s]', '\Delta\theta [deg]', ...
         '\Delta\beta [deg]', '\Delta p [deg/s]', '\Delta r [deg/s]', '\Delta\phi [deg]'};
scale = [1 r2d r2d r2d r2d r2d r2d r2d];

Phi = expm([A B; zeros(4, 12)]*dt);
Ad  = Phi(1:8, 1:8);
Bd  = Phi(1:8, 9:12);
uTrim = [s.dh s.da s.dr 0 0]*r2d;  uTrim(5) = s.dT;

fig = figure('Position', [100 100 1200 560]);
tl  = tiledlayout(fig, 2, 4, 'TileSpacing', 'compact');
for c = 1:2
    du = zeros(numel(t), 4);
    if c == 1, du(:,2) = dhP/r2d; panels = 1:4;
    else,      du(:,3) = daP/r2d; panels = 5:8; end

    xl = zeros(numel(t), 8);
    for k = 1:numel(t)-1
        xl(k+1,:) = (Ad*xl(k,:)' + Bd*du(k,:)')';
    end

    uExt = [t, repmat(uTrim, numel(t), 1)];
    uExt(:,2:3) = uExt(:,2:3) + du(:,2:3)*r2d;
    Y  = simFull(tr, idx, tEnd, uExt);
    yn = [Y.VT Y.alpha Y.q Y.theta Y.beta Y.p Y.r Y.phi];
    yn = interp1(Y.t, yn - yn(1,:), t);

    for i = panels
        nexttile(tl, i);
        plot(t, yn(:,i)*scale(i), '-', t, xl(:,i)*scale(i), '--', 'LineWidth', 1.2);
        grid on;  ylabel(names{i});
    end
end
xlabel(tl, 'Time [s]');
legend(nexttile(tl, 1), 'nonlinear', 'linear', 'Location', 'best');
end
