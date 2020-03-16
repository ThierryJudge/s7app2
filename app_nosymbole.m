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


%% Trajectoire Cubique

pi = [0,0,0];
pd = [0,0.01,0];
p_int = [pd(1)/2, pd(2)/2, 0.025];

dt = 1;
steps = 10;

positions1 = trajectoire_cubique(pi, p_int, dt/2, steps/2);
positions2 = trajectoire_cubique(p_int, pd, dt/2, steps/2);
positions = cat(1, positions1, positions2)


figure()
scatter3(positions(:, 1), positions(:, 2), positions(:, 3))
xlabel('X') 
ylabel('Y') 
zlabel('Z') 

%% Position initiale 

q_initial = [0 0.3 -aThigh aThigh+aTibia aTibia 0.3 0]';

%q_initial = [0 0 0.2 -0.4 -0.2 0]';
A0is = get_homo_mats(q_initial);
joint_positions = zeros(NB_LINKS, 3);

for i = 1:NB_LINKS+1
    p = squeeze(A0is(i, 1:3, 4))
    joint_positions(i, :) = p;
  
    joint_positions = round(joint_positions, 5)
end 

figure()
axis equal
hold on
plot3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3))
scatter3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3), 'O')
hold off
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
view(3);

position_initiale = squeeze(A0is(7, 1:3, 4))


%% Algo trajectoire 

q_initial = [0 0 0.2 -0.4 -0.2 0]';
A0is = get_homo_mats(q_initial);
pi = squeeze(A0is(7, 1:3, 4))

pd = pi
pd(2) = pd(2) - 0.01

%Interpolation 
p_int = [pd(1)/2, pd(2)/2, pi(3)+0.025];

dt = 1;
steps = 10;

positions1 = trajectoire_cubique(pi, p_int, dt/2, steps/2);
positions2 = trajectoire_cubique(p_int, pd, dt/2, steps/2);
positions = cat(1, positions1, positions2)

figure()
scatter3(positions(:, 1), positions(:, 2), positions(:, 3))
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
title('Interpolaton')


epsilon = 0.001;
K = 0.001;
k = 10 ^ -2;
q = q_initial';


joint_positions = zeros(length(positions), NB_LINKS, 3);

for interpolation_index = 1:length(positions)
    interpolation_index
    pd = squeeze(positions(interpolation_index, :))
    
    % Get current position
    A0is = get_homo_mats(q);
    pe = squeeze(A0is(7, 1:3, 4));
    e = pd - pe;
    disp(['Initial error: ',num2str(norm(e))])
    counter = 0;
    while norm(e) > epsilon 
        jac = algo_jaco(A0is, 0);
        jac = jac(1:3, :);
        delta_q = K * (jac' * (jac * jac' + k^2*eye(3, 3))^-1)*e';
        % move 
        q = double(q + delta_q);
        % recalculate error 
        A0is = get_homo_mats(q);
        pe = squeeze(A0is(7, 1:3, 4));
        e = pd - pe;
        if mod(counter,10) == 0
            disp(['Current error: ',num2str(norm(e))])
        end
        counter = counter + 1;
    end


    for i = 1:NB_LINKS+1
        p = squeeze(A0is(i, 1:3, 4));
        joint_positions(interpolation_index, i, :) = round(p, 5);
    end
    
    
    
end

figure()
axis equal
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
hold on
for i =1:length(positions)
    plot3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3))
    scatter3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3), 'O')
%     squeeze(joint_positions(i, 6, :))
end
view(3);
hold off 




