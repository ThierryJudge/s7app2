clear
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

%% Check a 0 

q = [0 0 0 0 0 0]';
A0is = get_homo_mats(q);

squeeze(A0is(end, :, :))

%% Position initiale 

q_initial = [0 0.3 -aThigh aThigh+aTibia aTibia 0.3]';

%q_initial = [0 0 0.2 -0.4 -0.2 0]';
A0is = get_homo_mats(q_initial);
joint_positions = zeros(NB_LINKS, 3);

size(A0is)

for i = 1:length(A0is)
    p = squeeze(A0is(i, 1:3, 4));
    joint_positions(i, :) = p;
  
    joint_positions = round(joint_positions, 5);
end 

figure()
title('Initial joint positions')
axis equal
hold on
plot3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3))
scatter3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3), 'O')
hold off
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
view(3);

position_initiale = squeeze(A0is(end, 1:3, 4));


%% Algo trajectoire 

% q_initial = [0 0 0.2 -0.4 -0.2 0]';
q_initial = [0 0.3 -aThigh aThigh+aTibia aTibia 0.3]';
A0is = get_homo_mats(q_initial);
p_initial = squeeze(A0is(end, 1:3, 4));

p_final = p_initial;
p_final(1) = p_final(1) + 0.03;
p_final(2) = p_final(2);

dt = 1;
steps = 50;

%Interpolation 
p_int = [(p_final(1) - p_initial(1))/2 + p_initial(1), (p_final(2) - p_initial(2))/2 + p_initial(2), p_initial(3)+0.025];

positions1 = trajectoire_cubique(p_initial, p_int, dt/2, steps/2);
positions2 = trajectoire_cubique(p_int, p_final, dt/2, steps/2);
positions = cat(1, positions1, positions2);


% Display 
figure()
axis equal
hold on
scatter3(positions(:, 1), positions(:, 2), positions(:, 3))
plot3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3))
scatter3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3), 'O')
hold off
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
title('Interpolaton')
view(3);

%% Run trajectory 
epsilon_p = 1e-10;
epsilon_r = 0.01;
K = 0.001;
k = 10 ^ -2;
q = q_initial';

kp = 0.5;
kr = 0.01;

joint_positions = zeros(length(positions), NB_LINKS, 3);

Re = squeeze(A0is(end,1:3,1:3)); % Current (initial) orientation 

Rd = Re; % GOAL orientation is initial orientation
nd = Rd(1:3,1);
sd = Rd(1:3,2);
ad = Rd(1:3,3);

for interpolation_index = 1:length(positions)
    disp(newline)
    disp(['interpolation_index: ',num2str(interpolation_index)])
    p_final = squeeze(positions(interpolation_index, :));
    
    % Get current position and error 
    A0is = get_homo_mats(q);
    pe = squeeze(A0is(end, 1:3, 4));
    ep = p_final - pe;
    
    
    % Get current otientation and error 
    Re = squeeze(A0is(end,1:3,1:3)); % Current orientation 
    ne = Re(1:3,1);
    se = Re(1:3,2);
    ae = Re(1:3,3);

    eo = 1/2 * (cross(ne, nd) + cross(se, sd) + cross(ae,ad));
    L = -1/2 * ((S(nd)*S(ne)) + (S(sd)*S(se)) + (S(ad)*S(ae)));

    er = inv(L)*eo;
    
    % Display 
    disp(['Goal position: ',num2str(p_final)])
    disp(['Initial position: ',num2str(pe)])
    disp(['Initial orientation: '])
    disp(Re)
    disp(['Initial position error: ',num2str(norm(ep))])
    disp(['Initial orientation error: ',num2str(norm(er))])
    counter = 0;
    
    while ((norm(ep) > epsilon_p) || norm(er) > epsilon_r) 
        jac = algo_jaco(A0is, 0);
        jac = correction * jac;
        
        Jp = jac(1:3, :);
        Jo = jac(4:6, :);
        
        delta_q = p_inv(Jp, kp)*kp*ep' + p_inv(hat(Jp, Jo), kr)*(kr*er - Jo*p_inv(Jp, kp)*kp*ep');

        %delta_q = K * p_inv(jac, k)*ep;

        % move 
        q = double(q + delta_q);
        
        
        %recalculate error 
        A0is = get_homo_mats(q);
        pe = squeeze(A0is(end, 1:3, 4));
        ep = p_final - pe;
        
        Re = squeeze(A0is(end,1:3,1:3)); % Current orientation 
        ne = Re(1:3,1);
        se = Re(1:3,2);
        ae = Re(1:3,3);

        eo = 1/2 * (cross(ne, nd) + cross(se, sd) + cross(ae,ad));
        L = -1/2 * ((S(nd)*S(ne)) + (S(sd)*S(se)) + (S(ad)*S(ae)));

        er = inv(L)*eo;
        
        % Display 
        if mod(counter,1000) == 0
             disp(['Index: ', num2str(interpolation_index), ', position error: ',num2str(norm(ep)), ', rotation error: ',num2str(norm(er)) ]);
        end
        counter = counter + 1;
    end

    disp(['Final position: ',num2str(pe)])
    for i = 1:length(A0is)
        p = squeeze(A0is(i, 1:3, 4));
        joint_positions(interpolation_index, i, :) = round(p, 5);
    end
    
    
    
end

%% Plot resutls 

final_positions = zeros(length(positions), 3);

figure()
title('Trajectory joint positions') 
for i =1:length(positions)
    plot3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3))
    xlabel('X') 
    ylabel('Y') 
    zlabel('Z')
    axis([-0.05 0.05 -0.05 0.05 -0.2 0.1])
    view(118,27)
    hold on
    scatter3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3), 'O')
    final_positions(i, :) = squeeze(joint_positions(i, end, :));
    scatter3(joint_positions(i, end, 1), joint_positions(i, end, 2), joint_positions(i, end, 3), '*', 'r')
    hold off
    pause(1/8)
end
view(3);
hold off 

figure()
title('Trajectory joint positions (1/2)')
axis equal
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
hold on
for i =1:length(positions)/2
    plot3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3))
    scatter3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3), 'O')
    final_positions(i, :) = squeeze(joint_positions(i, end, :));
    scatter3(joint_positions(i, end, 1), joint_positions(i, end, 2), joint_positions(i, end, 3), '*', 'r')
end
view(3);
hold off 


figure()
title('Trajectory joint positions (2/2)')
axis equal
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
hold on
for i =length(positions)/2:length(positions)
    plot3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3))
    scatter3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3), 'O')
    final_positions(i, :) = squeeze(joint_positions(i, end, :));
    scatter3(joint_positions(i, end, 1), joint_positions(i, end, 2), joint_positions(i, end, 3), '*', 'r')
end
view(3);
hold off 





figure()
axis equal
title('Effector positions')
xlabel('X') 
ylabel('Y') 
zlabel('Z')
hold on
for i =1:length(positions)
    final_positions(i, :)
    scatter3(final_positions(i, 1), final_positions(i, 2), final_positions(i, 3), '*')
end
view(3);
hold off 


% squeeze(positions(1, :))
% squeeze(positions(end, :))
% 
squeeze(joint_positions(1, end, :))
squeeze(joint_positions(end, end, :))


%% 

% x = [0 0.2 0.5 0.8 1]
% y = [0 0.25 1 0.25 0]
% 
% xq = 0:0.01:1
% vq = interpn(x,y,xq,'cubic');
% 
% figure()
% plot(x,y,'o',xq,vq,'-');
% 
% %%
% 
% x = [0:.1:1];
% y = normpdf(x,0.5,0.25) * 0.025;
% y = y - y(1);
% 
% figure()
% plot(x,y)

