%% Cleaning

clc
clear all
close all

%% Build DH table
thetas = sym('theta', [1 6]);
aThigh = 0.930;
aTibia = 0.930;
syms dThigh dTibia footheight;

dh = [0 0 -pi/2 thetas(1);
        0 0 -pi/2 thetas(2) + pi/2;
        aThigh 0 0 thetas(3);
        aTibia 0 pi thetas(4);
        0 0 -pi/2 thetas(5);
        footheight 0 0 thetas(6)];

homogeneous_matrix = get_homogeneous_matrix(dh);
p = get_jacobian_matrix(homogeneous_matrix)

%% Functions
function gjm = get_jacobian_matrix(HM)
    syms lorem;                     %% Use to instantiate matrices
    [n r c] = size(HM);
    tmp = zeros(4, 4) * lorem;
    
    p = zeros(n + 1, 3) * lorem;
    z = zeros(n + 1, 3) * lorem;
    
    z(1, :) = [0 0 1].';
    p(1, :) = [0 0 0].';
    
    result = zeros(n, 4, 4) * lorem;
    result(1, :, :) = HM(1, :, :);
    
    for i = 2 : n
        tmp(:, :) = HM(i, :, :);
        for j = 1 : i - 1
            t(:, :) = HM(i - j, :, :);
            tmp(:, :) = tmp(:, :) * t;
        end
        p(i + 1, :) = tmp(1:3, 4);
        z(i + 1, :) = tmp(1:3, 3);
        result(i, :, :) = tmp(:, :);
    end
    
    jacob = zeros(6, n) * lorem;
    pe = squeeze(p(n + 1, :));
    for i = 1 : n
        zi = squeeze(z(i, :));
        pi = squeeze(p(i, :));
        jacob(1:3, i) = cross(zi, pe - pi);
        jacob(4:6, i) = z(i);
    end
    
    gjm = jacob;
end

function ghm = get_homogeneous_matrix(M)
    syms lorem;     %% Use to instantiate matrices
    [r c] = size(M);  %% Number of links
    
    homogeneous_matrices = zeros(r, 4, 4) * lorem;
    for i = 1:r
        homogeneous_matrices(i, :, :) = build_homogeneous_matrix(M(i, 1:4));
    end
    
    ghm = simplify(homogeneous_matrices);
end

function bhm = build_homogeneous_matrix(B)
    temp = [cos(B(4)) (-sin(B(4)) * cos(B(2))) (sin(B(4)) * cos(B(2))) (B(1) * cos(B(4))); 
            sin(B(4)) (cos(B(4)) * cos(B(2))) (-cos(B(4)) * sin(B(2))) (B(1) * sin(B(4))); 
            0 sin(B(2)) cos(B(2)) B(3); 
            0 0 0 1];
    bhm = temp;
end