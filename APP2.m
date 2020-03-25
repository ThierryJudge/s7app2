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

% q_initial = [0 0 0 0 0 0] %Permet de valider le DH
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

x_max_delta = 0.03; %Valeur en x max
y_max_delta = 0.01; %Valeur en y max
z_max_value = 0.025;%Valeur en z max

p_final = p_initial;
p_final(1) = p_final(1) + x_max_delta;
p_final(2) = p_final(2) + y_max_delta;

dt = 2;         %Temps
steps = 20;     %Nombre de valeurs

%Interpolation 
p_int = [(p_final(1) - p_initial(1))/2 + p_initial(1), (p_final(2) - p_initial(2))/2 + p_initial(2), p_initial(3)+ z_max_value];

positions1 = trajectoire_cubique(p_initial, p_int, dt/2, steps/2); %Premiere cubique
positions2 = trajectoire_cubique(p_int, p_final, dt/2, steps/2);   %Deuxieme cubique
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
title('Position initial du pied avec interpolation souhaitée')

%% Run trajectory 

final_positions = zeros(length(positions), 3); %Initialisation

figure()
title('Trajectoire en temps réel')
axis equal
xlabel('X') 
ylabel('Y') 
zlabel('Z') 

epsilon_p = 0.0001; %Erreur min positon
epsilon_r = 0.0001; %Erreur min rotation

q = q_initial;

kp = 0.5;   %Gain position
kr = 0.01;  %Gain rotation

joint_positions = zeros(length(positions), NB_LINKS, 3); %Initialisation

Re = A0is(1:3,1:3,6); % Orientation (initial) courante

Rd = Re; % Orientation désiree (Le pied est deja parallele au sol. On doit conserver cette orientation.
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

counter_max = 1500

for interpolation_index = 1:length(positions)
    disp(newline)
    disp(['interpolation_index: ',num2str(interpolation_index)])
    p_final = squeeze(positions(interpolation_index, :)');
    
    % Get current position and error 
    [A0is, comp_homo_mat] = get_homo_mats(q);
    
    pe = comp_homo_mat(1:3, 4);
    ep = p_final - pe;
    
    
    % Get current otientation and error 
    Re = comp_homo_mat(1:3,1:3); % Orientation courante 
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
    
    

    while ((norm(ep) > epsilon_p) || norm(er) > epsilon_r) && (counter < counter_max) 
        jac = algo_jaco(A0is);
        Jp = jac(1:3, :);
        Jo = jac(4:6, :);
        
        delta_q = p_inv(Jp, kp)*kp*ep + p_inv(hat(Jp, Jo), kr)*(kr*er - Jo*p_inv(Jp, kp)*kp*ep);

        % position 
        q = double(q + delta_q);
        % recalculate error 
        [A0is, comp_homo_mat] = get_homo_mats(q);
        pe = comp_homo_mat(1:3, 4);
        ep = p_final - pe;
        
        % orientation
        Re = comp_homo_mat(1:3,1:3); % Orientation courante
        ne = Re(1:3,1);
        se = Re(1:3,2);
        ae = Re(1:3,3);
        % recalculate error 
        eo = 1/2 * (cross(ne, nd) + cross(se, sd) + cross(ae,ad));
        L = -1/2 * ((S(nd)*S(ne)) + (S(sd)*S(se)) + (S(ad)*S(ae)));
        er = inv(L)*eo;
        
        % Affichage des erreurs en position et rotation 
        if mod(counter,100) == 0
             disp(['Current position error: ',num2str(norm(ep)), ', index: ', num2str(interpolation_index), '/', num2str(length(positions))]);
             disp(['Current rotation error: ',num2str(norm(er)), ', index: ', num2str(interpolation_index), '/', num2str(length(positions))]);
        end
        counter = counter + 1;
    end
    
    disp(['Final rotation norm: '])
    disp(norm(er))
    
    disp(['Counter: ',num2str(counter)])
    
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

%Video
video = VideoWriter('APP2_Leg');
video.FrameRate = 30; %30 fps avec 60 valeurs = 2 sec = dt
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



