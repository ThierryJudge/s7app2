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

as = [0 0 dThigh dTibia 0 0];
ds = [0 0 0 0 0 0 ];
alphas = [-pi/2 -pi/2 0 pi pi/2 0];
thetas = [theta1 (theta2 - pi/2) theta3 theta4 theta5 theta6];

% Creation des matrices homogenes
syms syms_placeholder real
Ai_i1 = ones(NB_LINKS+1, 4, 4) * syms_placeholder; % Matrice du lien i a i+1
A0i = ones(NB_LINKS+1, 4, 4) * syms_placeholder; % Matrice du lien 0 a i

Tb = [0 1 0 hipOffsetX,
      1 0 0 hipOffsetY,
      0 0 -1 hipOffsetZ,
      0 0 0 1];  

tmp = Tb
  
for i=1:NB_LINKS
    mat = mat_homo(as(i), alphas(i), ds(i), thetas(i));
    Ai_i1(i, :, :) = mat;
    tmp = tmp*mat;
    A0i(i, :, :) = tmp;
end
    
  
Te = [0 0 -1 footHeight,
      0 -1 0 0,
      -1 0 0 0,
      0 0 0 1];   
  
A0i(7, :, :) = tmp * Te;
  





theta1_v = 0;
theta2_v = 0;
theta3_v = 0;
theta4_v = 0;
theta5_v = 0;
theta6_v = 0;
p = squeeze(A0i(7, 1:3, 4));
double(subs(p, {theta1, theta2, theta3, theta4, theta5, theta6}, {theta1_v, theta2_v, theta3_v, theta4_v, theta5_v, theta6_v}))

%% Plot robot


nb_links = 7;
joint_positions = zeros(nb_links, 3);

theta1_v = 0;
theta2_v = 0.3;
theta3_v = -aThigh;
theta4_v = aThigh+aTibia;
theta5_v = aTibia;
theta6_v = 0.3;

rad2deg(theta2_v)
rad2deg(theta3_v)
rad2deg(theta4_v)
rad2deg(theta5_v)
rad2deg(theta6_v)

% theta1_v = 0.78539;
% theta2_v = 0;
% theta3_v = 0;
% theta4_v = 0;
% theta5_v = 0;
% theta6_v = 0;

figure()
hold on
% for theta4_v = 0:0.1:0.4
for i = 1:nb_links
    i;
    squeeze(A0i(i, :, :));
    p = squeeze(A0i(i, 1:3, 4));
    p = subs(p, {theta1, theta2, theta3, theta4, theta5, theta6}, {theta1_v, theta2_v, theta3_v, theta4_v, theta5_v, theta6_v});
    double(p);
    joint_positions(i, :) = p;
end

joint_positions = round(joint_positions, 5);


axis equal
plot3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3))
scatter3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3), 'O')
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
view(3);
% end
hold off