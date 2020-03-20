close all
clear all
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 L1 L2 L3 L4 L5 L6 L7 real

q = [0; 0; 0; 0; 0; 0; 0];
theta1 = q(1);
theta2 = q(2);
theta3 = q(3);
theta4 = q(4);
theta5 = q(5);
theta6 = q(6);
theta7 = q(7);

[x_A01, y_A01, angle_A01, A01] = matrice_homogene(theta1, 1);
[x_A02, y_A02, angle_A02, A02] = matrice_homogene([theta1 theta2], [1 1]);
[x_A03, y_A03, angle_A03, A03] = matrice_homogene([theta1 theta2 theta3], [1 1 1]);
[x_A04, y_A04, angle_A04, A04] = matrice_homogene([theta1 theta2 theta3 theta4], [1 1 1 1]);
[x_A05, y_A05, angle_A05, A05] = matrice_homogene([theta1 theta2 theta3 theta4 theta5], [1 1 1 1 1]);
[x_A06, y_A06, angle_A06, A06] = matrice_homogene([theta1 theta2 theta3 theta4 theta5 theta6], [1 1 1 1 1 1]);
[x_A07, y_A07, angle_A07, A07] = matrice_homogene([theta1 theta2 theta3 theta4 theta5 theta6 theta7], [1 1 1 1 1 1 1]);

q0 = 0;

figure
hold on
plot([0 A01(13) A02(13) A03(13) A04(13) A05(13) A06(13) A07(13)], [0 A01(14) A02(14) A03(14) A04(14) A05(14) A06(14) A07(14)], '-o');

Pe = A07(1:3,end);
thetae = angle_A07;
q = q0;

tf = 2;
a0_x = Pe(1);
a0_y = Pe(2);
a1 = 0;
qf_x = 3;
qf_y = 3.5;
a2_x = (3*(qf_x - a0_x))/(tf^2);
a3_x = (-2 * a2_x)/(3 * tf);
a2_y = (3*(qf_y - a0_y))/(tf^2);
a3_y = (-2 * a2_y)/(3 * tf);

figure
hold on

for t=0:0.1:tf
    P_x = (a3_x*(t^3)) + (a2_x*(t^2)) + (a0_x);
    P_y = (a3_y*(t^3)) + (a2_y*(t^2)) + (a0_y);
    Pd = [P_x; P_y; 0;];
    ep = Pd - Pe;

    J = Ja(A01, A02, A03, A04, A05, A06, A07);
    Jp_q = J(1:3,:);
    Jo_q = J(4:6,:);
        
    kp = 10^-2;
    kr = 10^-3;
    Jo_q_inv = p_inv(Jo_q, kr, eye(3));

    Rd = eye(3);
    nd = Rd(1:3,1);
    sd = Rd(1:3,2);
    ad = Rd(1:3,3);

    Re = A07(1:3,1:3);
    ne = Re(1:3,1);
    se = Re(1:3,2);
    ae = Re(1:3,3);

    eo = 1/2 * (cross(ne, nd) + cross(se, sd) + cross(ae,ad));
    L = -1/2 * ((S(nd)*S(ne)) + (S(sd)*S(se)) + (S(ad)*S(ae)));

    er = inv(L)*eo;
    
    i = 0;
    i_max = 1000;

    while ((norm(ep) > 0.001) || norm(er) > 0.001) && (i < i_max) 
        
        J = Ja(A01, A02, A03, A04, A05, A06, A07);
        Jp_q = J(1:3,:);
        Jo_q = J(4:6,:);

        delta_q = p_inv(Jp_q, kp, eye(3))*kp*ep + p_inv(hat(Jp_q, Jo_q), kr, eye(3))*(kr*er - Jo_q*p_inv(Jp_q, kp, eye(3))*kp*ep);
        q = q + delta_q;

        [x_A01, y_A01, angle_A01, A01] = matrice_homogene(q(1), 1);
        [x_A02, y_A02, angle_A02, A02] = matrice_homogene([q(1) q(2)], [1 1]);
        [x_A03, y_A03, angle_A03, A03] = matrice_homogene([q(1) q(2) q(3)], [1 1 1]);
        [x_A04, y_A04, angle_A04, A04] = matrice_homogene([q(1) q(2) q(3) q(4)], [1 1 1 1]);
        [x_A05, y_A05, angle_A05, A05] = matrice_homogene([q(1) q(2) q(3) q(4) q(5)], [1 1 1 1 1]);
        [x_A06, y_A06, angle_A06, A06] = matrice_homogene([q(1) q(2) q(3) q(4) q(5) q(6)], [1 1 1 1 1 1]);
        [x_A07, y_A07, angle_A07, A07] = matrice_homogene([q(1) q(2) q(3) q(4) q(5) q(6) q(7)], [1 1 1 1 1 1 1]);

        Pe = A07(1:3,end);
        ep = Pd - Pe
        
        Re = A07(1:3,1:3);
        ne = Re(1:3,1);
        se = Re(1:3,2);
        ae = Re(1:3,3);

        eo = 1/2 * (cross(ne, nd) + cross(se, sd) + cross(ae,ad));
        L = -1/2 * ((S(nd)*S(ne)) + (S(sd)*S(se)) + (S(ad)*S(ae)));

        er = inv(L)*eo

        i = i+1;
    end
    plot([0 A01(13) A02(13) A03(13) A04(13) A05(13) A06(13) A07(13)], [0 A01(14) A02(14) A03(14) A04(14) A05(14) A06(14) A07(14)], '-o');
end

function dc = distance(p, pc, rc)
    x = abs(p(1) - pc(1) - rc)
    y = abs(p(2) - pc(2) - rc)
    
    dc = norm(x,y)
end

function J_inv = p_inv(J, k, I)
    J_inv = transpose(J)*(((J * transpose(J)) + (k^2 * I))^-1);
end

function X = S(w)
    X = [0 -w(3) w(2);
         w(3) 0 -w(1);
         -w(2) w(1) 0];
end

function H = hat(J1, J2)
    En = eye(7);
    H = J2*(En - pinv(J1)*J1);
end

 function [P_x, P_y, alpha, matrice] = matrice_homogene(theta, L)
    P_x = 0;
    P_y = 0;
    for k = 1:1:length(theta)
        L_k = L(k);
        theta_i = 0;
        for i = 1:1:k
            theta_i = theta_i + theta(i);
        end
        P_x = P_x + (L_k * cos(theta_i));
        P_y = P_y + (L_k * sin(theta_i));
    end
    
    matrice = [cos(sum(theta))   -sin(sum(theta))   0   P_x;
               sin(sum(theta))    cos(sum(theta))   0   P_y;
               0                  0                 1    0;
               0                  0                 0    1;];
          
    alpha = sum(theta);
 end

 function Jacobienne = Ja(A01, A02, A03, A04, A05, A06, A07)
    z0 = [0; 0; 1];
    z1 = A01(1:3,end-1);
    z2 = A02(1:3,end-1);
    z3 = A03(1:3,end-1);
    z4 = A04(1:3,end-1);
    z5 = A05(1:3,end-1);
    z6 = A06(1:3,end-1);

    P0 = [0; 0; 1];
    P1 = A01(1:3,end);
    P2 = A02(1:3,end);
    P3 = A03(1:3,end);
    P4 = A04(1:3,end);
    P5 = A05(1:3,end);
    P6 = A06(1:3,end);
    P7 = A07(1:3,end);

    %Calcul de la matrice jacobienne
    Jacobienne = [cross(z0, (P7 - P0)) cross(z1, (P7 - P1)) cross(z2, (P7 - P2)) cross(z3, (P7 - P3)) cross(z4, (P7 - P4)) cross(z5, (P7 - P5)) cross(z6, (P7 - P6))
          z0                   z1                   z2                   z3                   z4                   z5                    z6];
end