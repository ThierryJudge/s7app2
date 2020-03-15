function A0i = get_homo_mats(q)
    
hipOffsetY = .037; 
hipOffsetZ = .096; 
hipOffsetX = .008; 
thighLength = .0930; 
tibiaLength = .0930; 
footHeight = .0335;
kneeOffsetX = .04;

NB_LINKS = 6;

as = [0 0 thighLength tibiaLength 0 footHeight];
ds = [0 0 0 0 0 0 ];
alphas = [-pi/2 -pi/2 0 pi -pi/2 0];
% thetas = [theta1 theta2 + pi/2 theta3 theta4 theta5 theta6];
thetas = q;
thetas(2) = thetas(2) + pi/2;

% Creation des matrices homogenes
A0i = ones(NB_LINKS+1, 4, 4);

tmp = eye(4,4);

for i=1:NB_LINKS
    mat = mat_homo(as(i), alphas(i), ds(i), thetas(i));
    tmp = tmp*mat;
    A0i(i, :, :) = tmp;
end

% Correction pour le dernier join 
matrice_correction = [0 0 -1 hipOffsetX,
                      0 -1 0 hipOffsetY,
                      -1 0 0 hipOffsetZ,
                      0 0 0 1];                  
A0i(NB_LINKS+1, :, :) = matrice_correction * tmp;

end

