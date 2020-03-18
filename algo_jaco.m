function J = algo_jaco(matrices_homo, symbolic)

hipOffsetY = .037; 
hipOffsetZ = .096; 
hipOffsetX = .008; 

Tb = [0 1 0 hipOffsetX,
      1 0 0 -hipOffsetY,
      0 0 -1 hipOffsetZ,
      0 0 0 1];

j = length(matrices_homo);


J = ones(6, j);

z = zeros(j+1, 3);
p = zeros(j+1, 3);

if symbolic==1
    syms syms_placeholder real;
    J = J * syms_placeholder;
    z = z * syms_placeholder;
    p = p * syms_placeholder;
end

% z(1, :) = [0 0 1].';
% p(1, :) = [0 0 0].';

z(1, :) = Tb(1:3,3);
p(1, :) = Tb(1:3,4);

for i = 1:j
    tmp = squeeze(matrices_homo(i, :, :));
    z(i+1, :) = tmp(1:3, 3);
    p(i+1, :) = tmp(1:3, 4);
% 
% % Inserting in Jacobian
% 
pe = squeeze(p(j+1, :));

for i = 1:j
    zi = squeeze(z(i, :));
    pi = squeeze(p(i, :));
    J(1:3,i) = cross(zi, pe - pi);
    J(4:6,i) = zi;
end 

end

