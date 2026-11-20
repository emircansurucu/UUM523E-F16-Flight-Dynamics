function Z = sim_F16(tr, tEnd, uExt)
%SIM_F16  F16_6DOF'u trim noktasından başlatıp benzetir.
%
%   Z = sim_F16(tr, tEnd)        kumandalar trim değerinde sabit
%   Z = sim_F16(tr, tEnd, uExt)  uExt = [t, dh da dr dsb throttle] (deg, %)
%
%   Rijit cisim durumları trimden, eyleyiciler trim açısında, motor gücü gaz
%   kolunun kararlı gücünde ve LEF filtresi kararlı halde başlar. dlef model
%   içinde tarifeyi izler.
%
%   Z sütunları (SI, rad, dT %; Midterm Sim_* düzeni):
%     t VT alpha beta p q r phi theta psi xN yE h dT dh da dr

r2d = 180/pi;  s = tr.s;
M = f16_modelInit(tr.xcg);
[~, x0] = f16_eval(M, s);
F16_6DOF([], [], [], 'term');
i = M.idx;
if nargin < 3
    u = [s.dh s.da s.dr 0 0]*r2d;  u(5) = s.dT;
    uExt = [0 u; tEnd u];
end
assignin('base', 'x0_sim', x0);
assignin('base', 'u_sim', uExt);
so = sim('F16_6DOF', 'StopTime', num2str(tEnd), 'MaxStep', '0.01', ...
         'LoadInitialState', 'on', 'InitialState', 'x0_sim', ...
         'LoadExternalInput', 'on', 'ExternalInput', 'u_sim', ...
         'SaveState', 'on', 'StateSaveName', 'xout', 'SaveFormat', 'Array');
X   = so.xout;
uvw = X(:, i.uvw);
VT  = sqrt(sum(uvw.^2, 2));
dT  = interp1(uExt(:,1), uExt(:,6), so.tout);
Z = [so.tout, VT, atan2(uvw(:,3), uvw(:,1)), asin(uvw(:,2)./VT), X(:, i.pqr), X(:, i.euler), ...
     X(:, i.pos(1:2)), -X(:, i.pos(3)), dT, X(:, [i.dh i.da i.dr])/r2d];
[~, iu] = unique(so.tout);       % değişken adımlı çözücü aynı anı iki kez yazabilir
Z = Z(iu, :);
end
