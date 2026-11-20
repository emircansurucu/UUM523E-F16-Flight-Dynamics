function [A, B] = linearize_F16(M, tr, eksen, lef)
%LINEARIZE_F16  Trim noktası etrafında sayısal doğrusallaştırma (merkezi fark).
%
%   [A, B] = linearize_F16(M, tr, eksen, lef)
%
%   eksen = 'body' -> x = [u w q theta v p r phi]
%   eksen = 'wind' -> x = [VT alpha q theta beta p r phi]
%   u = [dT dh da dr]  (dT %, yüzeyler rad)
%   lef   = 'tarife' -> dlef her adımda tarifeden (alpha ve qbar/p_s ile) hesaplanır
%   lef   = 'sabit'  -> dlef trim değerinde tutulur (Midterm Problem 2)
%
%   Her durum ve giriş sırayla +-adım kadar değiştirilir, türevler modelden
%   okunur. Bu 8 durumlu modelde eyleyici ve motor dinamiği yok; h ve psi
%   modele girmez.

s = tr.s;
VT = sqrt(s.u^2 + s.v^2 + s.w^2);
if strcmp(eksen, 'body')
    x0 = [s.u; s.w; s.q; s.theta; s.v; s.p; s.r; s.phi];
else
    x0 = [VT; atan2(s.w, s.u); s.q; s.theta; asin(s.v/VT); s.p; s.r; s.phi];
end
u0 = [s.dT; s.dh; s.da; s.dr];
% Ders 5'teki gibi bütün durum ve girişlerde adım 1e-3 (m/s, rad, rad/s, %).
% Bu adım trimdeki sayısal beta artığından (~1e-7) çok büyük: tablolar
% beta = 0'da kırılıyor, merkezi fark iki yanın ortalamasını almalı.
hx = 1e-3*ones(8, 1);
hu = 1e-3*ones(4, 1);

A = zeros(8, 8);
B = zeros(8, 4);
for k = 1:8
    dx = zeros(8, 1);  dx(k) = hx(k);
    A(:,k) = (turev(M, tr, x0 + dx, u0, eksen, lef) - turev(M, tr, x0 - dx, u0, eksen, lef))/(2*hx(k));
end
for k = 1:4
    du = zeros(4, 1);  du(k) = hu(k);
    B(:,k) = (turev(M, tr, x0, u0 + du, eksen, lef) - turev(M, tr, x0, u0 - du, eksen, lef))/(2*hu(k));
end
end

function xd = turev(M, tr, x, u, eksen, lef)
s = tr.s;
if strcmp(eksen, 'body')
    s.u = x(1);  s.w = x(2);  s.v = x(5);
else
    s.u = x(1)*cos(x(2))*cos(x(5));
    s.v = x(1)*sin(x(5));
    s.w = x(1)*sin(x(2))*cos(x(5));
end
s.q = x(3);  s.theta = x(4);  s.p = x(6);  s.r = x(7);  s.phi = x(8);
s.dT = u(1);  s.dh = u(2);  s.da = u(3);  s.dr = u(4);

if strcmp(lef, 'tarife')
    % h sabit olduğundan qbar/p_s = VT^2/(2 R T) sadece VT ile değişir
    VT = sqrt(s.u^2 + s.v^2 + s.w^2);
    qps = tr.c.qps*(VT/tr.c.VT)^2;
    s.dlef = lefTarife(atan2(s.w, s.u), qps);
end

d = f16_eval(M, s);
if strcmp(eksen, 'body')
    xd = [d.udot; d.wdot; d.qdot; d.thetadot; d.vdot; d.pdot; d.rdot; d.phidot];
else
    xd = [d.VTdot; d.alphadot; d.qdot; d.thetadot; d.betadot; d.pdot; d.rdot; d.phidot];
end
end
