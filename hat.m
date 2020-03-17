function H = hat(J1, J2)
    x = size(J1, 2);
    En = eye(x);
    H = J2*(En - pinv(J1)*J1);
end

