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

NB_LINKS = 6;

pospiedz = hipOffsetZ - dThigh - dTibia - footHeight;

% q_initial = [0 0 0 0 0 0]
q_initial = [0 0.3 -aThigh aThigh+aTibia aTibia 0.3];
  
[A0is, mat_homo] = get_homo_mats(q_initial);


% Position initiale 

for i = 1:NB_LINKS
    p = A0is(1:3, 4, i);
    joint_positions(i, :) = p;
end 

joint_positions(7, :) = mat_homo(1:3, 4);

% Algo trajectoire 
q_initial = [0 0.3 -aThigh aThigh+aTibia aTibia 0.3]';
p_initial = mat_homo(1:3, 4)';

p_final = p_initial;
p_final(1) = p_final(1) + 0.03;
p_final(2) = p_final(2);

dt = 2;
steps = 60;

%Interpolation 
p_int = [(p_final(1) - p_initial(1))/2 + p_initial(1), (p_final(2) - p_initial(2))/2 + p_initial(2), p_initial(3)+0.025];

positions1 = trajectoire_cubique(p_initial, p_int, dt/2, steps/2);
positions2 = trajectoire_cubique(p_int, p_final, dt/2, steps/2);
positions = cat(1, positions1, positions2);


% Display 
figure()
axis equal
hold on
axis([-0.10 0.10 -0.10 0.10 -0.15 0.1])
view(118,27)
plot3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3))
scatter3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3), 'O')
scatter3(positions(:, 1), positions(:, 2), positions(:, 3))
hold off
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
title('Position initial du pied avec interpolation souhait�e')

%% Run trajectory 
% epsilon_p = 1e-10;

final_positions = zeros(length(positions), 3);

figure()
title('Trajectoire en temps r�el')
axis equal
xlabel('X') 
ylabel('Y') 
zlabel('Z') 

epsilon_p = 0.00001;
epsilon_r = 0.01;
K = 0.001;
k = 10 ^ -2;
q = q_initial;

kp = 0.5;
kr = 0.01;

joint_positions = zeros(length(positions), NB_LINKS, 3);

Re = A0is(1:3,1:3,6); % Current (initial) orientation 

Rd = Re; % GOAL orientation is initial orientation
nd = Rd(1:3,1);
sd = Rd(1:3,2);
ad = Rd(1:3,3);

R_u = [0 1 0 0 0 0,
       1 0 0 0 0 0,
       0 0 -1 0 0 0,
       0 0 0 0 1 0,
       0 0 0 1 0 0,
       0 0 0 0 0 -1];
   
qs = zeros(steps, NB_LINKS)

for interpolation_index = 1:length(positions)
    disp(newline)
    disp(['interpolation_index: ',num2str(interpolation_index)])
    p_final = squeeze(positions(interpolation_index, :)');
    
    % Get current position and error 
    [A0is, comp_homo_mat] = get_homo_mats(q);
    
%     pe = A0is(1:3, 4, end);
    pe = comp_homo_mat(1:3, 4);
    ep = p_final - pe;
    
    
    % Get current otientation and error 
%     Re = A0is(1:3,1:3,6); % Current orientation 
    Re = comp_homo_mat(1:3,1:3); % Current orientation 
    ne = Re(1:3,1);
    se = Re(1:3,2);
    ae = Re(1:3,3);

    eo = 1/2 * (cross(ne, nd) + cross(se, sd) + cross(ae,ad));
    L = -1/2 * ((S(nd)*S(ne)) + (S(sd)*S(se)) + (S(ad)*S(ae)));

    er = inv(L)*eo;
    
    % Display 
    disp(['Goal position: ',num2str(p_final')])
    disp(['Initial position: ',num2str(pe')])
    disp(['Initial orientation: '])
    disp(Re)
    disp(['Initial position error: ',num2str(norm(ep))])
    disp(['Initial orientation error: ',num2str(norm(er))])
    counter = 0;

    while ((norm(ep) > epsilon_p) || norm(er) > epsilon_r) 
        jac = algo_jaco(A0is);
        Jp = jac(1:3, :);
        Jo = jac(4:6, :);
        
        delta_q = p_inv(Jp, kp)*kp*ep + p_inv(hat(Jp, Jo), kr)*(kr*er - Jo*p_inv(Jp, kp)*kp*ep);
  %     delta_q = p_inv(Jp, kp)*kp*ep' + p_inv(hat(Jp, Jo), kr)*(kr*er - Jo*p_inv(Jp, kp)*kp*ep');
        %delta_q = K * p_inv(jac, k)*ep;

        % move 
        q = double(q + delta_q);
        
        
        %recalculate error 
        [A0is, comp_homo_mat] = get_homo_mats(q);
%         pe = A0is(1:3, 4, 6);
        pe = comp_homo_mat(1:3, 4);
        ep = p_final - pe;
        
%         Re = A0is(1:3,1:3, 6); % Current orientation 
        Re = comp_homo_mat(1:3,1:3); % Current orientation 
        ne = Re(1:3,1);
        se = Re(1:3,2);
        ae = Re(1:3,3);

        eo = 1/2 * (cross(ne, nd) + cross(se, sd) + cross(ae,ad));
        L = -1/2 * ((S(nd)*S(ne)) + (S(sd)*S(se)) + (S(ad)*S(ae)));

        er = inv(L)*eo;
        
        % Display 
        if mod(counter,100) == 0
             disp(['Current position error: ',num2str(norm(ep)), ', index: ', num2str(interpolation_index), '/', num2str(length(positions))]);
             disp(['Current rotation error: ',num2str(norm(er)), ', index: ', num2str(interpolation_index), '/', num2str(length(positions))]);
        end
        counter = counter + 1;
    end
    
%     qs(interpolation_index, :) = q;
    qs(interpolation_index, :) = round(q, 4);

    disp(['Final position: ',num2str(pe')])
    for i = 1:NB_LINKS
        p = A0is(1:3, 4, i);
        joint_positions(interpolation_index, i, :) = round(p, 5);
    end
    
   joint_positions(interpolation_index, 7, :) = comp_homo_mat(1:3, 4);
   
    hold on
    axis([-0.10 0.10 -0.10 0.10 -0.15 0.1])
    view(118,27)
    plot3(joint_positions(interpolation_index, :, 1), joint_positions(interpolation_index, :, 2), joint_positions(interpolation_index, :, 3))
    scatter3(joint_positions(interpolation_index, :, 1), joint_positions(interpolation_index, :, 2), joint_positions(interpolation_index, :, 3), 'O')
    final_positions(interpolation_index, :) = squeeze(joint_positions(interpolation_index, end, :));
    xlabel('X') 
    ylabel('Y') 
    zlabel('Z') 
    pause(1/30)
    hold off
    
end

% view(3);
hold off 

%% Output joint positions to txt file
time = 2;
dt = time/steps;
t = 0:dt:time-dt;
timed_qs = [t', qs];

dlmwrite('qs.txt',timed_qs,'delimiter',',','newline','pc')
disp('File qs.txt')


%% Plot resutls 

% figure()
% title('Trajectory joint positions')
% axis equal
% xlabel('X') 
% ylabel('Y') 
% zlabel('Z') 
% hold on
% for i =1:length(positions)
%     plot3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3))
%     scatter3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3), 'O')
%     final_positions(i, :) = squeeze(joint_positions(i, end, :));
%     scatter3(joint_positions(i, end, 1), joint_positions(i, end, 2), joint_positions(i, end, 3), '*', 'r')
% end
% view(3);
% hold off 

%Video
video = VideoWriter('APP2_Leg');
video.FrameRate = 30;
open(video)

length(positions)

figure()
axis equal
hold on
for i =1:length(positions)

    plot3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3))
    xlabel('X') 
    ylabel('Y') 
    zlabel('Z')
    hold on
    axis([-0.10 0.10 -0.10 0.10 -0.15 0.1])
    view(118,27)
    title('Animation de la jambe')
    scatter3(joint_positions(i, :, 1), joint_positions(i, :, 2), joint_positions(i, :, 3), 'O')
    final_positions(i, :) = squeeze(joint_positions(i, end, :));
    hold off
    pause(1/8)
    frame = getframe(gcf);
    writeVideo(video, frame);
end
hold off 

close(video)


