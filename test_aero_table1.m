%% test_aero_table1.m
%  F16_6DOF aero alt sistemini (Aerodynamics) TEK BASINA test eder.
%
%  Yontem:
%   - Gecici bir test modeli olusturulur; F16_6DOF/Aerodynamics icine kopyalanir.
%     (F16_6DOF.slx DEGISTIRILMEZ.)
%   - 16 girise Constant bloklari baglanir; HW-1 Tablo 1'deki 10 durum sirayla
%     verilir. Ucus kosulu: 0.4 Mach, 10000 ft (atmosfer: SLJ App. A.6, SI).
%   - Cikislar (F_aero, M_aero) okunur, katsayilar F/(qS), M/(qSb), M/(qSc)
%     ile geri hesaplanir.
%   - Sonuclar bagimsiz referansla karsilastirilir (asagida).
%
%  Birimler (aero alt sistemi girisleri):
%   alpha, beta, dh, dlef, d_sb, da, dr : deg
%   p, q, r : rad/s      TAS : m/s      q_dyn : Pa
%   S_ref : m^2   b_ref, c_ref : m     x_cg : c-bar orani
%
%  Calistirma:  >> test_aero_table1

run(fullfile(fileparts(mfilename('fullpath')), 'init_F16.m'));   % bu klasordeki init

%% Ucus kosulu: 0.4 Mach, 10000 ft (Stevens-Lewis-Johnson App. A.6, SI)
h     = h_val_m;
Tfac  = 1 - lapse_SI*h;
Tatm  = T0_SI*Tfac;   if h >= h_trop_SI, Tatm = T_iso_SI; end
rho   = R0_SI*Tfac^4.14;
a_snd = sqrt(gamma_R_SI*Tatm);
V     = Mach_val*a_snd;
qbar  = 0.5*rho*V^2;
fprintf('Ucus kosulu: V = %.4f m/s, qbar = %.3f Pa, rho = %.5f kg/m^3\n', V, qbar, rho);

%% HW-1 Tablo 1 (her satir bir durum)
%        alpha  beta    p      q     r     da      dh     dr    dlef  dsb   [deg, deg/s]
cases = [ 0     0       0      0     0     0       0      0     0     0;
          0     0       0      0     0     0       0      0     0     60;
          20    0       0      0     0     0       0      0     0     0;
          0     12.5    0      0     0     0       0      0     0     0;
          0     0       0      0     0     0      -15     0     0     0;
          0     0       0      0     0     0       0      0     25    0;
          0     0       0      0     0     10      0      0     0     0;
          0     0       0      0     0     0       0     -15    0     0;
          5.16  0       0      0     0     0      -0.47   0     7.55  0;
          5.95  0      -0.53   1.21  2.09 -0.055  -0.62  -0.33  8.65  0 ];

%% Gecici test modeli
tm = 'aero_test_harness';
if bdIsLoaded(tm), close_system(tm, 0); end
load_system('F16_6DOF');
new_system(tm);
add_block('F16_6DOF/Aerodynamics', [tm '/Aero'], 'Position', [300 50 450 650]);

% Giris sirasi = Aerodynamics port sirasi
inNames = {'alpha','beta','dh','dlef','q','d_sb','da','dr','r','p', ...
           'x_cg','q_dyn','S_ref','b_ref','c_ref','TAS'};
for k = 1:numel(inNames)
    y = 40 + 37*k;
    blk = add_block('simulink/Sources/Constant', [tm '/u_' inNames{k}], ...
            'Value', ['u.' inNames{k}], 'Position', [120 y 200 y+20]);
    ph = get_param(blk, 'PortHandles');  ah = get_param([tm '/Aero'], 'PortHandles');
    add_line(tm, ph.Outport(1), ah.Inport(k));
end
ah = get_param([tm '/Aero'], 'PortHandles');
for k = 1:2
    ob = add_block('simulink/Sinks/Out1', [tm '/y' num2str(k)], ...
            'Position', [550 200+200*(k-1) 580 214+200*(k-1)]);
    oph = get_param(ob, 'PortHandles');
    add_line(tm, ah.Outport(k), oph.Inport(1));
end
set_param(tm, 'StopTime','0', 'SaveOutput','on', 'OutputSaveName','yout', ...
              'SaveFormat','Array', 'ReturnWorkspaceOutputs','on');

%% 10 durumu calistir
AeroResults = zeros(10, 12);   % CX CY CZ Cl Cm Cn X Y Z L M N
for i = 1:10
    c = cases(i,:);
    u = struct('alpha',c(1), 'beta',c(2), 'p',c(3)*deg2rad, 'q',c(4)*deg2rad, ...
               'r',c(5)*deg2rad, 'da',c(6), 'dh',c(7), 'dr',c(8), 'dlef',c(9), ...
               'd_sb',c(10), 'x_cg',xcg, 'q_dyn',qbar, 'S_ref',S, 'b_ref',b, ...
               'c_ref',cbar, 'TAS',V);
    so = sim(tm);
    yo = so.get('yout');
    F = yo(end, 1:3);  M = yo(end, 4:6);
    AeroResults(i,:) = [F/(qbar*S), M(1)/(qbar*S*b), M(2)/(qbar*S*cbar), ...
                        M(3)/(qbar*S*b), F, M];
end
close_system(tm, 0);

%% Bagimsiz referans
%  Hocanin verisi + TP-1538 Ek B ve HW-1 Cm,t formulleriyle ayrica (Python,
%  scipy) hesaplandi; Clp(55) = -0.12 duzeltmesi dahil.
ref = [
  -2.0200000000e-02  0.0000000000e+00 -1.0000000000e-01  0.0000000000e+00 -7.7000000000e-03  0.0000000000e+00 -4.3948999594e+03  0.0000000000e+00 -2.1756930492e+04  0.0000000000e+00 -5.7802914806e+03  0.0000000000e+00;
  -3.0300000000e-02  0.0000000000e+00 -4.8580000000e-01  0.0000000000e+00 -1.1100000000e-02  0.0000000000e+00 -6.5923499391e+03  0.0000000000e+00 -1.0569516833e+05  0.0000000000e+00 -8.3326279785e+03  0.0000000000e+00;
   2.2100000000e-02  0.0000000000e+00 -1.3550000000e+00  0.0000000000e+00  2.3900000000e-02  0.0000000000e+00  4.8082816388e+03  0.0000000000e+00 -2.9480640817e+05  0.0000000000e+00  1.7941424206e+04  0.0000000000e+00;
  -1.5200000000e-02 -2.6770000000e-01 -1.0650000000e-01 -1.8200000000e-02 -6.6000000000e-03  5.1200000000e-02 -3.3070534348e+03 -5.8243302928e+04 -2.3171130974e+04 -3.6208057781e+04 -4.9545355548e+03  1.0186003068e+05;
  -5.0166666667e-02  0.0000000000e+00  1.9333333333e-02  0.0000000000e+00  1.2773333333e-01  0.0000000000e+00 -1.0914726797e+04  0.0000000000e+00  4.2063398952e+03  0.0000000000e+00  9.5887779019e+04  0.0000000000e+00;
  -4.8900000000e-02  0.0000000000e+00 -2.5000000000e-02  0.0000000000e+00 -4.0800000000e-02  0.0000000000e+00 -1.0639139011e+04  0.0000000000e+00 -5.4392326230e+03  0.0000000000e+00 -3.0628037975e+04  0.0000000000e+00;
  -2.0200000000e-02  1.0450000000e-02 -1.0000000000e-01 -2.5600000000e-02 -7.7000000000e-03 -4.9500000000e-03 -4.3948999594e+03  2.2735992364e+03 -2.1756930492e+04 -5.0930015340e+04 -5.7802914806e+03 -9.8477959348e+03;
  -2.0200000000e-02 -4.1050000000e-02 -1.0000000000e-01 -7.2500000000e-03 -7.7000000000e-03  2.2550000000e-02 -4.3948999594e+03 -8.9312199670e+03 -2.1756930492e+04 -1.4423539500e+04 -5.7802914806e+03  4.4862181481e+04;
  -3.9603904000e-03  0.0000000000e+00 -4.1721748800e-01  0.0000000000e+00  6.1060000000e-05  0.0000000000e+00 -8.6165938655e+02  0.0000000000e+00 -9.0773718865e+04  0.0000000000e+00  4.5836960754e+01  0.0000000000e+00;
   6.2113792274e-04  1.9329135835e-04 -4.7855827871e-01  2.8712307165e-04  5.3804529415e-05  3.2871918880e-05  1.3514054611e+02  4.2054266483e+01 -1.0411959206e+05  5.7121806419e+02  4.0390371817e+01  6.5397161438e+01 ];

%% Rapor
names = {'CX','CY','CZ','Cl','Cm','Cn','X','Y','Z','L','M','N'};
fprintf('\nModel katsayilari (Tablo 1):\n');
fprintf('%4s', 'case'); fprintf('%12s', names{1:6}); fprintf('\n');
for i = 1:10
    fprintf('%4d', i); fprintf('%12.6f', AeroResults(i,1:6)); fprintf('\n');
end

d = abs(AeroResults - ref);
tolC = 1e-6;  tolF = 1e-2;          % katsayi / kuvvet-moment toleransi
bad = [d(:,1:6) > tolC, d(:,7:12) > tolF];
fprintf('\nReferansla en buyuk mutlak fark:\n');
for j = 1:12
    [mx, ic] = max(d(:,j));
    flag = ''; if any(bad(:,j)), flag = '   <-- FARKLI'; end
    fprintf('  %-3s %.3e  (durum %d)%s\n', names{j}, mx, ic, flag);
end
if any(bad(:))
    [ri, cj] = find(bad);
    fprintf(2, '\n%d hucrede fark var. Ornek: durum %d, %s: model %.6g, referans %.6g\n', ...
            numel(ri), ri(1), names{cj(1)}, AeroResults(ri(1),cj(1)), ref(ri(1),cj(1)));
else
    disp('Tum 10 durum referansla uyumlu. AeroResults calisma alaninda.');
end
