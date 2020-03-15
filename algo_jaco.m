function J = algo_jaco(matrices_homo, symbolic)

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

z(1, :) = [0 0 1].';
p(1, :) = [0 0 0].';

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

