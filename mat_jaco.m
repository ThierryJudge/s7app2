function J = mat_jaco(T,Tb)

%     T(1:3,4,0)
    I = Tb;
    nbNode = length(T);
    for j=1:1:nbNode
        ref = j-1
        if (ref>0)
            Jp = cross(T(1:3,3,ref), T(1:3,4,end)-T(1:3,4,ref));
            Jo = T(1:3,3,ref);
        else
            Jp = cross(I(1:3,3), (T(1:3,4,end)-I(1:3,4)));
            Jo = I(1:3,3);
        end

        J(:,j) = [Jp;Jo]
    end
end