function positions = trajectoire_cubique(pi, pd, dt , steps)
%trajectoire_cubique 
%   pi: (3,), position initial (x,y,z)
%   pi: (3,), position finale (x,y,z)
%   dt: temps de la trajectoire
%   steps: nombre de points a prendre.

a0 = pi; % position initiale
a1 = [0,0,0]; %vitesse finale 
a2 = (pd-a0)./ (dt^2/3);
a3 = (-2/(3 * dt)) * a2;

positions = zeros(steps, 3);
for i = 1:steps
    t = dt/steps * i;
    positions(i, :) = a3 * t^3 + a2 * t^2 + a0;
end
end

