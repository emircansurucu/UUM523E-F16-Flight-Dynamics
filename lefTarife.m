function dlef = lefTarife(alpha, qps)
%LEFTARIFE  Otomatik hücum kenarı flapı tarifesi (kararlı hal), rad.
%   dlef = 1.38*alpha - 9.05*qbar/p_s + 1.45  [deg], 0-25 deg arasında
%   Modeldeki tarifenin (2s+7.25)/(s+7.25) filtresi kararlı halde 1'dir.
dlef = min(max(1.38*alpha*180/pi - 9.05*qps + 1.45, 0), 25)*pi/180;
end
