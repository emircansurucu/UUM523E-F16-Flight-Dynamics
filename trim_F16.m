function tr = trim_F16(M, c)
%TRIM_F16  Düz uçuş ve kararlı dönüş için trim (fmincon).
%
%   tr = trim_F16(M, c)
%
%   M : f16_modelInit ile derlenmiş model
%   c : uçuş koşulu
%       VT [m/s], h [m], qps (qbar/p_s), gamma [rad]
%       beta, phi [rad], nz [g]  -> NaN ise belirtilmemiş, trimde bulunur
%       turn  -> true: kararlı dönüş (psidot serbest), false: düz uçuş (psidot = 0)
%
%   Bilinmeyenler: z = [alpha beta phi theta psidot dT dh da dr]
%   (açılar rad, psidot rad/s, dT 0-1 arası)
%
%   Dönüş kinematiği (Stevens-Lewis): p = -psidot*sin(theta),
%   q = psidot*sin(phi)*cos(theta), r = psidot*cos(phi)*cos(theta).
%   Böylece phidot = thetadot = 0 olur.
%
%   Maliyet fonksiyonu (Stevens-Lewis-Johnson):
%     J = VTdot^2 + 100*(alphadot^2 + betadot^2) + 10*(pdot^2 + qdot^2 + rdot^2)
%   Eşitlik kısıtları: uçuş yolu açısı ve (verilmişse) nz.

d2r = pi/180;

%      alpha     beta      phi       theta     psidot  dT   dh        da          dr
lb = [-10*d2r, -30*d2r, -90*d2r, -90*d2r, -1.5,   0, -25*d2r, -21.5*d2r, -30*d2r];
ub = [ 45*d2r,  30*d2r,  90*d2r,  90*d2r,  1.5,   1,  25*d2r,  21.5*d2r,  30*d2r];

% Başlangıç tahmini
phi0 = 0;
if ~isnan(c.phi), phi0 = c.phi; end
if ~isnan(c.nz),  phi0 = acos(1/c.nz); end
psid0 = 0;
if c.turn, psid0 = M.g*tan(phi0)/c.VT; end
z0 = [3*d2r, 0, phi0, 3*d2r + c.gamma, psid0, 0.3, -2*d2r, 0, 0];

% Verilen büyüklükler alt ve üst sınır eşitlenerek sabitlenir
if ~isnan(c.beta), lb(2) = c.beta; ub(2) = c.beta; z0(2) = c.beta; end
if ~isnan(c.phi),  lb(3) = c.phi;  ub(3) = c.phi;  z0(3) = c.phi;  end
if ~c.turn,        lb(5) = 0;      ub(5) = 0;      end

opt = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', ...
    'MaxFunctionEvaluations', 2e4, 'MaxIterations', 2e3, ...
    'OptimalityTolerance', 1e-12, 'ConstraintTolerance', 1e-10, 'StepTolerance', 1e-14);
[z, J, exitflag] = fmincon(@(z) maliyet(M, c, z), z0, [], [], [], [], lb, ub, ...
                           @(z) kisit(M, c, z), opt);

s = durum(c, z);
d = f16_eval(M, s);
[~, ceq] = kisit(M, c, z);

tr.s   = s;
tr.d   = d;
tr.J   = J;
tr.ceq = ceq;
tr.exitflag = exitflag;
tr.xcg = M.xcg;

% TrimResults satırı: VT alpha beta phi theta psi(psidot) p q r dT dh da dr dlef J
psiCol = s.psi;
if c.turn, psiCol = d.psidot; end
tr.row = [c.VT, z(1), z(2), z(3), z(4), psiCol, s.p, s.q, s.r, ...
          s.dT, s.dh, s.da, s.dr, s.dlef, J];
end

function s = durum(c, z)
alpha = z(1); beta = z(2); phi = z(3); theta = z(4); psidot = z(5);
s.u = c.VT*cos(alpha)*cos(beta);
s.v = c.VT*sin(beta);
s.w = c.VT*sin(alpha)*cos(beta);
s.p = -psidot*sin(theta);
s.q =  psidot*sin(phi)*cos(theta);
s.r =  psidot*cos(phi)*cos(theta);
s.phi = phi;  s.theta = theta;  s.psi = 0;
s.h  = c.h;
s.dT = 100*z(6);
s.dh = z(7);  s.da = z(8);  s.dr = z(9);
% Otomatik LEF tarifesi (kararlı hal), 0-25 deg arasında
s.dlef = min(max(1.38*alpha/pi*180 - 9.05*c.qps + 1.45, 0), 25)*pi/180;
end

function J = maliyet(M, c, z)
d = f16_eval(M, durum(c, z));
J = d.VTdot^2 + 100*(d.alphadot^2 + d.betadot^2) + 10*(d.pdot^2 + d.qdot^2 + d.rdot^2);
end

function [cin, ceq] = kisit(M, c, z)
alpha = z(1); beta = z(2); phi = z(3); theta = z(4);
a = cos(alpha)*cos(beta);
b = sin(phi)*sin(beta) + cos(phi)*sin(alpha)*cos(beta);
cin = [];
ceq = sin(c.gamma) - (a*sin(theta) - b*cos(theta));
if ~isnan(c.nz)
    d = f16_eval(M, durum(c, z));
    ceq(2) = d.nz - c.nz;
end
end
