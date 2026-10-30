%% test_thrust_table2.m
%  F16_6DOF motor alt sistemini (F16_6DOF/Propulsion) TEK BASINA test eder.
%  Gecici bir test modeli kurulur; F16_6DOF.slx DEGISTIRILMEZ.
%
%  Girisler : 1 throttle_pct [%]   2 mach [-]   3 altitude [m]
%  Cikis    : F_thrust = [T 0 0] [N]  (kararli hal: P3_0 = P1, t = 0 aninda)
%
%  HW-1 Tablo 2 (PDF'te irtifa sutunu bir satir kaymis; dogru okuma asagida).
%
%  Calistirma:  >> test_thrust_table2

run(fullfile(fileparts(mfilename('fullpath')), 'init_F16.m'));   % bu klasordeki init

%        Mach   irtifa [ft]  gaz kolu [%]
cases = [0.25    5000        10;
         0.65   25000        77;
         0.90   45000        95;
         0.40   10000        16.66;
         0.40   10000        19.77];

%% Gecici test modeli
tm = 'thrust_test_harness';
if bdIsLoaded(tm), close_system(tm, 0); end
load_system('F16_6DOF');
new_system(tm);
add_block('F16_6DOF/Propulsion', [tm '/Engine'], 'Position', [300 50 420 200]);
inNames = {'thr','mach','alt_m'};
ah = get_param([tm '/Engine'], 'PortHandles');
for k = 1:3
    y = 40 + 50*k;
    blk = add_block('simulink/Sources/Constant', [tm '/u_' inNames{k}], ...
            'Value', ['u.' inNames{k}], 'Position', [120 y 200 y+20]);
    ph = get_param(blk, 'PortHandles');
    add_line(tm, ph.Outport(1), ah.Inport(k));
end
ob = add_block('simulink/Sinks/Out1', [tm '/y'], 'Position', [500 115 530 129]);
oph = get_param(ob, 'PortHandles');
add_line(tm, ah.Outport(1), oph.Inport(1));
set_param(tm, 'StopTime','0', 'SaveOutput','on', 'OutputSaveName','yout', ...
              'SaveFormat','Array', 'ReturnWorkspaceOutputs','on');

%% 5 durumu calistir
ThrustResults = zeros(5, 1);
for i = 1:5
    u = struct('thr',cases(i,3), 'mach',cases(i,1), 'alt_m',cases(i,2)*ft2m);
    P3_0 = interp1(thr_BP, P1_tab, cases(i,3));   % motor kararli halde: P3 = P1
    so = sim(tm);
    yo = so.get('yout');
    ThrustResults(i) = yo(end, 1);
end
close_system(tm, 0);

%% Bagimsiz referans
%  Hocanin motor verisi + TP-1538 Fig. 66 (disli 0/77/100 -> 0/50/100,
%  idle-mil-max ara degerleme) ile ayrica (Python) hesaplandi.
ref = [7.8918029517e+03; 2.6867260880e+04; 2.4059659194e+04; 9.0493253463e+03; 1.0717845909e+04];

%% Rapor
fprintf('\n%4s %6s %8s %8s %14s %14s %10s\n', 'case','Mach','h [ft]','thr [%]','T model [N]','T ref [N]','fark [N]');
for i = 1:5
    fprintf('%4d %6.2f %8d %8.2f %14.4f %14.4f %10.2e\n', i, cases(i,1), cases(i,2), cases(i,3), ...
            ThrustResults(i), ref(i), ThrustResults(i) - ref(i));
end
if all(abs(ThrustResults - ref) < 1e-2)
    disp('Tum 5 durum referansla uyumlu. ThrustResults calisma alaninda.');
else
    fprintf(2, 'Referansla fark var (tolerans 0.01 N).\n');
end
