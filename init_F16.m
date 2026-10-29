%% =========================================================================
%  UUM 523E - HOMEWORK #1: F-16 PLANT MODEL INITIALIZATION (SI UNITS)
%  Kaynaklar:
%    1. NASA TP-1538 (Nguyen et al., 1979)
%    2. Stevens, Lewis, and Johnson (2016) - Appendix A.6
%% =========================================================================

%% 1. Veri Dosyalarinin Yuklenmesi (.mat)
addpath('data');
load("f16_AerodynamicData.mat")
load('f16_EngineData.mat');

%% 2. Aerodinamik Tablo Kısayolları
AeroData = f16_AerodynamicData.data;
AeroBP   = f16_AerodynamicData.BP;

%% Veri düzeltmesi: ders verisinde Clp(alpha = 55 deg) = -1200 girilmiş.
% NASA TP-1538 Tablo III'te bu değer -0.12'dir (komşu degerler -0.10 / -0.14).
iClp = AeroBP.Cl.Clp.alpha == 55;
if AeroData.Cl.Clp(iClp) < -100
    AeroData.Cl.Clp(iClp) = -0.12;
    fprintf('init_F16: Clp(alpha=55) -1200 -> -0.12 olarak düzeltildi (TP-1538).\n');
end

% dh = 0 deg dilimleri
CX_dh0 = AeroData.CX.CX(:,:,3);
CZ_dh0 = AeroData.CZ.CZ(:,:,3);
Cm_dh0 = AeroData.Cm.Cm(:,:,3);
Cn_dh0 = AeroData.Cn.Cn(:,:,2);
Cl_dh0 = AeroData.Cl.Cl(:,:,2);

%% 3. Birim Dönüşüm Katsayıları
ft2m   = 0.3048;                 % 1 ft = 0.3048 m
lbf2N  = 4.448222;               % 1 lbf = 4.448222 N
deg2rad = pi / 180;
rad2deg = 180 / pi;
slug2kg = lbf2N / ft2m;
R_conv = (ft2m^2) * 1.8;

% Atmosfer sabitleri (Stevens-Lewis-Johnson App. A.6 -> SI)
R0_SI = 2.377e-3 * (slug2kg / ft2m^3);
lapse_SI = 0.703e-5 / ft2m;
T0_SI = 519.0 / 1.8;
T_iso_SI = 390 / 1.8;
h_trop_SI = 35000 * ft2m;
gamma_R_SI = 1.4 * 1716.3 * R_conv;
R_ps_SI = 1715 * R_conv;

%% 4. Kütle, Geometri ve Atalet Özellikleri (NASA TP-1538 Tablo I -> SI)
g      = 9.80665;                % Yercekimi ivmesi (m/s^2)
Weight = 20500.0 * lbf2N;        % Ağırlık: 91188.551 N
m      = Weight / g;             % Kütle: ~9298.64 kg (637.14 slugs)

b      = 30.0 * ft2m;            % Kanat açıklığı: 9.144 m
S      = 300.0 * (ft2m^2);       % Kanat alanı: 27.870912 m^2
cbar   = 11.32 * ft2m;           % Ortalama aerodinamik veter (MAC): 3.450336 m

xcgr   = 0.35;                   % Referans c.g. konumu (0.35 * cbar)
xcg    = 0.35;                   % Gerçek c.g. konumu (0.35 * cbar)

% Atalet Momentleri (slug*ft^2 -> kg*m^2 dönüşümü: 1 slug = lbf*s^2/ft)
slugft2_to_kgm2 = lbf2N * ft2m;  % ~1.355818
Ixx = 9496.0  * slugft2_to_kgm2; % ~12874.85 kg*m^2
Iyy = 55814.0 * slugft2_to_kgm2; % ~75673.62 kg*m^2
Izz = 63100.0 * slugft2_to_kgm2; % ~85552.11 kg*m^2
Ixz = 982.0   * slugft2_to_kgm2; % ~1331.41 kg*m^2

I_mat = [ Ixx,  0,   -Ixz;
          0,    Iyy,  0;
         -Ixz,  0,    Izz ];
inv_I = inv(I_mat);

% Motor Açısal Momentum Sabiti
He = 160.0 * slugft2_to_kgm2;    % ~216.93 kg*m^2/s

% Baslangıç Durum Vektörü
V_body_0 = [131.28; 0; 0];
omega_0 = [0;0;0];
Euler_0 = [0;0;0];
Pos_0 = [0;0;-3048];

%% 5. Tablo 1 Doğrulama Uçuş Kosulu Sabitleri (0.4 Mach, 10000 ft)
h_val_ft = 10000.0;
h_val_m  = h_val_ft * ft2m;      % 3048 m
Mach_val = 0.4;

%% 6. Motor (Propulsion) Tabloları
EngData = f16_EngineData.data.Thrust_lbs;
EngBP   = f16_EngineData.BP;

% Gaz kolu Dişlisi (NASA TP-1538 Fig. 66b): gaz kolu konumu [%] -> güç komutu P1 [%]
% %77 = military (P1 = 50), %100 = maximum (P1 = 100)
thr_BP    = [0, 77, 100];
P1_tab    = [0, 50, 100];

% Zaman sabiti tablosu (NASA TP-1538 Fig. 66c: dP = P2 - P3 -> 1/tau_T)
dP_BP        = [-100, 25, 50, 100];
inv_tauT_tab = [1.0, 1.0, 0.1, 0.1];

% Motor güç seviyesi P3 baslangıç degeri [%] (Propulsion/Engine Dynamics integratörü).
% Kararlı hal için gaz kolunun P1 degeri verilir; trimde güncellenir.
P3_0 = 0;

disp('F-16 SI Parametreleri ve NASA TP-1538 Veri Tabloları Başarıyla Yüklendi.');