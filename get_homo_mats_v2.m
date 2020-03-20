function [A0is, comp_mat_homo] = get_homo_mats_v2(q)
    
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

pospiedz = hipOffsetZ - dThigh - dTibia - footHeight;

NB_LINKS = 6;

as = [0 0 dThigh dTibia 0 0];
ds = [0 0 0 0 0 0 ];
alphas = [-pi/2 -pi/2 0 pi pi/2 0];
% q_initial = [0 0 0 0 0 0];
thetas = q;
thetas(2) = thetas(2) - pi/2;

% Creation des matrices homogenes
A0is = ones(4, 4, NB_LINKS);

Tb = [0 1 0 hipOffsetX,
      1 0 0 -hipOffsetY,
      0 0 -1 hipOffsetZ,
      0 0 0 1]; 

% tmp = eye(4:4);

tmp = Tb;

for i=1:NB_LINKS
    mat = mat_homo(as(i), alphas(i), ds(i), thetas(i));
    tmp = tmp*mat;
    A0is(:, :, i) = tmp;
end


Te = [0 0 -1 footHeight,
      0 -1 0 0,
      -1 0 0 0,
      0 0 0 1]; 
  
  
comp_mat_homo = tmp*Te;

end

