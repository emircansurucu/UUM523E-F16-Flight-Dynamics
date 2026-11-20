%% run_HW2.m
%  UUM 523E - HW-2 (Grup 2). F-16 trim ve doğrusallaştırma.
%
%   1. Problem 1: Tablo 1'deki 6 trim durumu (fmincon)    -> TrimResults (6x15)
%   2. Problem 2: gövde ve rüzgâr ekseninde doğrusal model -> A_Body, B_Body, A_Wind, B_Wind
%      (koşul i, ii, iii; dlef tarifeyi izler)
%   3. Sonuç dosyası: HW2_Group2.mat (sadece bu 5 değişken)
%   4. Kontrol: trimdeki artıklar, doğrusal ve doğrusal olmayan yanıt
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
    Tatm = T0_SI*(1 - lapse_SI*h);                 % bütün durumlar troposferde
    V    = caseTab(k,1)*sqrt(gamma_R_SI*Tatm);
    cases(k).VT    = V;
    cases(k).h     = h;
    cases(k).qps   = 0.5*V^2/(R_ps_SI*Tatm);       % qbar/p_s = (rho V^2/2)/(R rho T)
    cases(k).gamma = caseTab(k,3)*d2r;
    cases(k).beta  = caseTab(k,4)*d2r;
    cases(k).phi   = caseTab(k,5)*d2r;
    cases(k).nz    = caseTab(k,6);
    cases(k).turn  = caseTab(k,7) == 1;
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
        [A_Body(:,:,j), B_Body(:,:,j)] = linearize_F16(M, tr, 'body', 'tarife');
        [A_Wind(:,:,j), B_Wind(:,:,j)] = linearize_F16(M, tr, 'wind', 'tarife');
    end
    F16_6DOF([], [], [], 'term');
end

colNames = {'VT','alpha','beta','phi','theta','psi/psid','p','q','r','dT[%]','dh','da','dr','dlef','J'};
fprintf('\nProblem 1 - TrimResults (SI, rad, rad/s, dT %%):\n%5s', 'durum');
fprintf('%12s', colNames{:});  fprintf('\n');
for k = 1:nCase, fprintf('%5d', k);  fprintf('%12.5g', TrimResults(k,:));  fprintf('\n'); end

fprintf('\nTrim özeti (deg):\n%5s %8s %8s %8s %8s %8s %8s %8s %8s %8s %8s %9s %5s\n', 'durum', ...
        'alpha', 'beta', 'phi', 'theta', 'mu', 'dh', 'da', 'dr', 'dlef', 'dT[%]', '1/cos(mu)', 'exit');
for k = 1:nCase
    fprintf('%5d %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %9.4f %5d\n', k, ...
            TrimResults(k,2:5)*r2d, trims{k}.mu*r2d, TrimResults(k,11:14)*r2d, TrimResults(k,10), ...
            1/cos(trims{k}.mu), trims{k}.exitflag);
end

%% 2. Problem 2 - Doğrusal modeller
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

fprintf('\nÖzdeğerler (rüzgâr ekseni A):\n');
for j = 1:3
    fprintf('  %-4s boylamsal (A11): %s\n', linName{j}, eigStr(eig(A_Wind(1:4,1:4,j))));
    fprintf('       yanal (A22):     %s\n', eigStr(eig(A_Wind(5:8,5:8,j))));
    fprintf('       tam 8x8:         %s\n', eigStr(eig(A_Wind(:,:,j))));
end

%% 3. Sonuç dosyası
resFile = fullfile(here, sprintf('HW2_Group%d.mat', groupNo));
save(resFile, 'TrimResults', 'A_Body', 'B_Body', 'A_Wind', 'B_Wind');
fprintf('\nKaydedildi: %s\n', resFile);

%% 4. Kontrol
% 4a. Trimde modelin türevleri (konum ve psi hariç) ve kısıtlar sıfır olmalı
fprintf('\nKontrol - trimdeki artıklar (en büyük mutlak değer):\n');
fprintf('%5s %12s %12s %12s %12s\n', 'durum', 'uvw [m/s2]', 'pqr [r/s2]', 'eyl/motor', 'kısıt');
for k = 1:nCase
    d = trims{k}.d;
    fprintf('%5d %12.2e %12.2e %12.2e %12.2e\n', k, max(abs([d.udot d.vdot d.wdot])), ...
            max(abs([d.pdot d.qdot d.rdot d.phidot d.thetadot])), max(abs(d.aux)), max(abs(trims{k}.ceq)));
end

% 4b. Doğrusal ve doğrusal olmayan yanıt (koşul i ve iii), 10 s.
%     Üst satır: stabilatör ikilisi (1-2 s +, 2-3 s -), alt satır: kanatçık darbesi (1-2 s).
%     Koşul iii'te trimde dlef = 1.14 deg; alpha 0.8 deg azalırsa flap 0 sınırına
%     dayanır ve kararlılaştırıcı etkisi kaybolur. Bu yüzden genlik 0.1 deg.
amp = [1 0 0.1];     % deg
for j = [1 3]
    tr  = trims{linCase(j)};
    fig = linNonlin(tr, A_Wind(:,:,j), B_Wind(:,:,j), 10, amp(j));
    title(fig.Children(1), sprintf(['Condition %s (x_{cg} = %.2f c): %.1f deg \\delta_h doublet (top), ' ...
          '%.1f deg \\delta_a pulse (bottom)'], linName{j}, tr.xcg, amp(j), amp(j)));
    exportgraphics(fig, fullfile(figDir, ['HW2_LinVsNonlin_' linName{j} '.png']), 'Resolution', 200);
    close(fig);
end
fprintf('\nGrafikler: %s\n', figDir);

%% Yardımcı fonksiyonlar
function printMat(name, X, rowN, colN)
fprintf('\n%s:\n%8s', name, '');  fprintf('%12s', colN{:});  fprintf('\n');
for i = 1:size(X,1), fprintf('%8s', rowN{i});  fprintf('%12.4g', X(i,:));  fprintf('\n'); end
end

function str = eigStr(e)
e = sort(e);
str = strjoin(arrayfun(@(z) sprintf('%.4g%+.4gj', real(z), imag(z)), e, 'UniformOutput', false), ', ');
end

function fig = linNonlin(tr, A, B, tEnd, amp)
% Doğrusal model (rüzgâr ekseni) sıfır dereceli tutucu ile ayrıklaştırılır,
% tam model aynı girişlerle koşturulur. Yanıtlar trimden sapma olarak çizilir.
s   = tr.s;  r2d = 180/pi;
dt  = 0.01;  t = (0:dt:tEnd)';
dhP = amp*((t >= 1 & t < 2) - (t >= 2 & t < 3));   % deg
daP = amp*(t >= 1 & t < 2);                         % deg
names = {'\Delta V_T [m/s]', '\Delta\alpha [deg]', '\Delta q [deg/s]', '\Delta\theta [deg]', ...
         '\Delta\beta [deg]', '\Delta p [deg/s]', '\Delta r [deg/s]', '\Delta\phi [deg]'};
scale = [1 r2d r2d r2d r2d r2d r2d r2d];
col   = [2 3 6 9 4 5 7 8];      % [VT alpha q theta beta p r phi] -> sim_F16 sütunları

Phi = expm([A B; zeros(4, 12)]*dt);
Ad  = Phi(1:8, 1:8);
Bd  = Phi(1:8, 9:12);
uTrim = [s.dh s.da s.dr 0 0]*r2d;  uTrim(5) = s.dT;

fig = figure('Visible', 'off', 'Position', [100 100 1200 560]);
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
    Z  = sim_F16(tr, tEnd, uExt);
    yn = interp1(Z(:,1), Z(:,col) - Z(1,col), t);

    for i = panels
        nexttile(tl, i);
        plot(t, yn(:,i)*scale(i), '-', t, xl(:,i)*scale(i), '--', 'LineWidth', 1.2);
        grid on;  ylabel(names{i});
    end
end
xlabel(tl, 'Time [s]');
legend(nexttile(tl, 1), 'nonlinear', 'linear', 'Location', 'best');
end
