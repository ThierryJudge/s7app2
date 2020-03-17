function Jinv = p_inv(J, k)
    x = size(J, 1);
    Jinv = J' * ((J * J' + k^2*eye(x, x))^-1);

end

