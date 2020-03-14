clear all
close all
clc

syms dThigh dTibia footHeight real
syms hipOffsetX hipOffsetY hipOffsetZ real
syms theta1 theta2 theta3 theta4 theta5 theta6 real

NB_LINKS = 6;

%Parametres DH 
as = [0 0 dThigh dTibia 0 footHeight];
ds = [0 0 0 0 0 0 ];
alphas = [-pi/2 -pi/2 0 pi -pi/2 0];
thetas = [theta1 theta2 + pi/2 theta3 theta4 theta5 theta6];

syms syms_placeholder real
Ai_i1 = ones(NB_LINKS+1, 4, 4) * syms_placeholder; % Matrice du lien i a i+1
A0i = ones(NB_LINKS+1, 4, 4) * syms_placeholder; % Matrice du lien 0 a i

tmp = eye(4,4);




for i=1:NB_LINKS
    mat = mat_homo(as(i), alphas(i), ds(i), thetas(i));
    Ai_i1(i, :, :) = mat;
    tmp = mat * tmp;
    A0i(i, :, :) = tmp;
end

% Correction pour le dernier join 
matrice_correction = [0 0 -1 hipOffsetX,
                      0 -1 0 hipOffsetY,
                      -1 0 0 hipOffsetZ,
                      0 0 0 1];                  
Ai_i1(NB_LINKS+1, :, :) = matrice_correction;
A0i(NB_LINKS+1, :, :) = matrice_correction * tmp;



matrice_jacobienne = algo_jaco(A0i);
matrice_jacobienne
size(matrice_jacobienne)

%%

pi = [0,0,0];
pd = [1,1,0];
dt = 1;
steps = 10;

positions = trajectoire_cubique(pi, pd, dt, steps)

figure()
scatter(positions(:, 1), positions(:, 2))





