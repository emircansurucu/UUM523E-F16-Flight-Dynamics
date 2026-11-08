function [d, x, u] = f16_eval(M, s)
%F16_EVAL  Verilen durum ve kumandalar için F16_6DOF modelinin türevleri.
%
%   [d, x, u] = f16_eval(M, s)
%
%   s alanları (SI, açılar rad): u v w p q r phi theta psi h
%                                dT [%], dh da dr dlef [rad]
%
%   Eyleyiciler komut edilen açıda, motor gücü P3 gaz kolunun kararlı
%   gücünde ve LEF filtresi kararlı halde başlatılır. Türevler doğrudan
%   modelden okunur: F16_6DOF(0, x, u, 'derivs').

r2d = 180/pi;
i = M.idx;
alpha = atan2(s.w, s.u);

x = zeros(M.nx, 1);
x(i.uvw)   = [s.u; s.v; s.w];
x(i.pqr)   = [s.p; s.q; s.r];
x(i.euler) = [s.phi; s.theta; s.psi];
x(i.pos)   = [0; 0; -s.h];
x(i.dh)    = s.dh*r2d;
x(i.da)    = s.da*r2d;
x(i.dr)    = s.dr*r2d;
x(i.dlef)  = s.dlef*r2d;
x(i.P3)    = interp1(M.thr_BP, M.P1_tab, s.dT);
x(i.lefTF) = alpha*r2d/7.25;          % (2s+7.25)/(s+7.25) filtresinin kararlı hali
u = [s.dh*r2d; s.da*r2d; s.dr*r2d; 0; s.dT];

F16_6DOF(0, x, u, 'outputs');
xd = F16_6DOF(0, x, u, 'derivs');

d.xdot = xd;
d.udot = xd(i.uvw(1));  d.vdot = xd(i.uvw(2));  d.wdot = xd(i.uvw(3));
d.pdot = xd(i.pqr(1));  d.qdot = xd(i.pqr(2));  d.rdot = xd(i.pqr(3));
d.phidot   = xd(i.euler(1));
d.thetadot = xd(i.euler(2));
d.psidot   = xd(i.euler(3));
d.aux = xd([i.dh i.da i.dr i.dlef i.lefTF i.P3]);   % trimde sıfır olmalı

% Rüzgâr ekseni türevleri (FORMULLER.md, Bölüm 2)
VT = sqrt(s.u^2 + s.v^2 + s.w^2);
d.VTdot    = (s.u*d.udot + s.v*d.vdot + s.w*d.wdot)/VT;
d.alphadot = (s.u*d.wdot - s.w*d.udot)/(s.u^2 + s.w^2);
d.betadot  = (d.vdot*VT - s.v*d.VTdot)/(VT^2*cos(asin(s.v/VT)));

% Yük faktörü: gövde -z yönündeki aero + itki kuvvetinin ağırlığa oranı
% (wdot = q*u - p*v + g*cos(phi)*cos(theta) + Z/m  denkleminden)
d.nz = -(d.wdot - s.q*s.u + s.p*s.v - M.g*cos(s.phi)*cos(s.theta))/M.g;
end
