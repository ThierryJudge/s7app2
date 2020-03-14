clear all
close all
clc

hipOffsetY = .037; 
hipOffsetZ = .096; 
hipOffsetX = .008; 
thighLength = .0930; 
tibiaLength = .0930; 
footHeight = .0335;
kneeOffsetX = .04;

dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
aThigh = atan(kneeOffsetX/thighLength);
dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
aTibia = atan(kneeOffsetX/tibiaLength);

%syms dThigh dTibia footHeight real
%syms hipOffsetX hipOffsetY hipOffsetZ real
syms theta1 theta2 theta3 theta4 theta5 theta6 real

NB_LINKS = 6;


%% Parametres DH 

as = [0 0 dThigh dTibia 0 footHeight];
ds = [0 0 0 0 0 0 ];
alphas = [-pi/2 -pi/2 0 pi -pi/2 0];
thetas = [theta1 theta2 + pi/2 theta3 theta4 theta5 theta6];

% Creation des matrices homogenes
syms syms_placeholder real
Ai_i1 = ones(NB_LINKS+1, 4, 4) * syms_placeholder; % Matrice du lien i a i+1
A0i = ones(NB_LINKS+1, 4, 4) * syms_placeholder; % Matrice du lien 0 a i

tmp = eye(4,4);

for i=1:NB_LINKS
    mat = mat_homo(as(i), alphas(i), ds(i), thetas(i));
    Ai_i1(i, :, :) = mat;
    tmp = tmp*mat;
    A0i(i, :, :) = tmp;
end

% Correction pour le dernier join 
matrice_correction = [0 0 -1 hipOffsetX,
                      0 -1 0 hipOffsetY,
                      -1 0 0 hipOffsetZ,
                      0 0 0 1];                  
Ai_i1(NB_LINKS+1, :, :) = matrice_correction;
A0i(NB_LINKS+1, :, :) = matrice_correction * tmp;

%% Creation de la matrice jacobienne 

matrice_jacobienne = algo_jaco(A0i);
matrice_jacobienne
size(matrice_jacobienne)

%% Plot robot

joint_positions = zeros(NB_LINKS, 3);

theta1_v = 0;
theta2_v = 0.3;
theta3_v = -aThigh;
theta4_v = aThigh+aTibia;
theta5_v = aTibia;
theta6_v = 0.3;

for i = 1:NB_LINKS
    i
    squeeze(A0i(i, :, :))
    p = squeeze(A0i(i, 1:3, 4));
    p
    p = subs(p, {theta1, theta2, theta3, theta4, theta5, theta6}, {theta1_v, theta2_v, theta3_v, theta4_v, theta5_v, theta6_v});
    double(p)
    joint_positions(i, :) = p;
end

joint_positions

figure()
hold on
plot3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3))
scatter3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3), 'O')
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
view(3);
%hold off


%% Trajectoire Cubique

pi = [0,0,0];
pd = [1,1,1];
dt = 1;
steps = 10;

positions = trajectoire_cubique(pi, pd, dt, steps)

figure()
scatter3(positions(:, 1), positions(:, 2), positions(:, 3))





