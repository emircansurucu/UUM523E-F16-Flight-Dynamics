function [A, B] = linearize_F16(M, tr, eksen)
%LINEARIZE_F16  Trim noktası etrafında sayısal doğrusallaştırma (merkezi fark).
%
%   [A, B] = linearize_F16(M, tr, eksen)
%
%   eksen = 'body' -> x = [u w q theta v p r phi]
%   eksen = 'wind' -> x = [VT alpha q theta beta p r phi]
%   u = [dT dh da dr]  (dT %, yüzeyler rad)
%
%   Her durum ve giriş sırayla +-adım kadar değiştirilir, türevler modelden
%   okunur. Bu 8 durumlu modelde eyleyici ve motor dinamiği yok; dlef trim
%   değerinde sabit tutulur, h ve psi modele girmez.

s = tr.s;
VT = sqrt(s.u^2 + s.v^2 + s.w^2);
if strcmp(eksen, 'body')
    x0 = [s.u; s.w; s.q; s.theta; s.v; s.p; s.r; s.phi];
    hx = 1e-4*[VT VT 1 1 VT 1 1 1]';
else
    x0 = [VT; atan2(s.w, s.u); s.q; s.theta; asin(s.v/VT); s.p; s.r; s.phi];
    hx = 1e-4*[VT 1 1 1 1 1 1 1]';
end
u0 = [s.dT; s.dh; s.da; s.dr];
hu = 1e-4*[100 1 1 1]';
% Adımlar trimdeki küçük sayısal artıklardan (beta ~ 1e-7) çok büyük seçildi:
% tablolar beta = 0'da kırılıyor, merkezi fark iki yanın ortalamasını almalı.

A = zeros(8, 8);
B = zeros(8, 4);
for k = 1:8
    dx = zeros(8, 1);  dx(k) = hx(k);
    A(:,k) = (turev(M, s, x0 + dx, u0, eksen) - turev(M, s, x0 - dx, u0, eksen))/(2*hx(k));
end
for k = 1:4
    du = zeros(4, 1);  du(k) = hu(k);
    B(:,k) = (turev(M, s, x0, u0 + du, eksen) - turev(M, s, x0, u0 - du, eksen))/(2*hu(k));
end
end

function xd = turev(M, s, x, u, eksen)
if strcmp(eksen, 'body')
    s.u = x(1);  s.w = x(2);  s.v = x(5);
else
    s.u = x(1)*cos(x(2))*cos(x(5));
    s.v = x(1)*sin(x(5));
    s.w = x(1)*sin(x(2))*cos(x(5));
end
s.q = x(3);  s.theta = x(4);  s.p = x(6);  s.r = x(7);  s.phi = x(8);
s.dT = u(1);  s.dh = u(2);  s.da = u(3);  s.dr = u(4);

d = f16_eval(M, s);
if strcmp(eksen, 'body')
    xd = [d.udot; d.wdot; d.qdot; d.thetadot; d.vdot; d.pdot; d.rdot; d.phidot];
else
    xd = [d.VTdot; d.alphadot; d.qdot; d.thetadot; d.betadot; d.pdot; d.rdot; d.phidot];
end
end
