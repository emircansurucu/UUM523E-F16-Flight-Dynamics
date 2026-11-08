function M = f16_modelInit(xcg)
%F16_MODELINIT  F16_6DOF modelini verilen ağırlık merkezi için derler.
%
%   M = f16_modelInit(xcg)
%
%   Trim ve doğrusallaştırma HW-1 modelini model API'si ile çağırır:
%   F16_6DOF(t, x, u, 'derivs'). Model dosyası değiştirilmez; sadece xcg
%   değişkeni güncellenip model yeniden derlenir. init_F16 önceden
%   çalıştırılmış olmalı. İş bitince F16_6DOF([], [], [], 'term') çağrılır.

assignin('base', 'xcg', xcg);

% Durum vektöründe her büyüklüğün yeri (vektör durumlar art arda gelir)
[~, ~, str] = F16_6DOF([], [], [], 0);
yer = @(blok) find(strcmp(str, ['F16_6DOF/' blok]));
M.idx.uvw   = yer('Equations of Motion/Integrator');      % [u v w] m/s
M.idx.pqr   = yer('Equations of Motion/Integrator1');     % [p q r] rad/s
M.idx.euler = yer('Equations of Motion/Integrator2');     % [phi theta psi] rad
M.idx.pos   = yer('Equations of Motion/Integrator3');     % [x y z] NED, m
M.idx.dh    = yer('Actuators/Integrator');                % deg
M.idx.da    = yer('Actuators/Integrator1');               % deg
M.idx.dr    = yer('Actuators/Integrator2');               % deg
M.idx.dlef  = yer('Actuators/Integrator3');               % deg
M.idx.lefTF = yer('Transfer Fcn');                        % LEF ön-geri filtresi
M.idx.P3    = yer('Propulsion/Engine Dynamics/P3_int');   % motor gücü, %
M.nx  = numel(str);
M.xcg = xcg;

% Gaz dişlisi ve yerçekimi (init_F16'daki değerler)
M.thr_BP = evalin('base', 'thr_BP');
M.P1_tab = evalin('base', 'P1_tab');
M.g      = evalin('base', 'g');

F16_6DOF([], [], [], 'compile');
end
