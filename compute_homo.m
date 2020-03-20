function [T, p] = compute_homo(Tb,A,Te)
    T(:,:,1) = Tb*A(:,:,1);
    p(:,:,1) = T(1:3,4,1);
    [dump1,dump2,nbNode] = size(A);
    for i=2:1:nbNode
      T(:,:,i) = T(:,:,i-1)*A(:,:,i);
      p(:,:,i) = T(1:3,4,i);
    end
    T(:,:,end) = T(:,:,end)*Te;
    p(:,:,end) = T(1:3,4,end);
end