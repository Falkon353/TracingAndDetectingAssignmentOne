function [e, eRes] = computeEnergi(rVtEst, intrinsicMatrix,Mi,mi);
[N,M] = size(Mi);
R = rotationVectorToMatrix(rVtEst(1:3))';
t = rVtEst(4:6)';
e = 0;
eRes = zeros(2*N,1);
for i = 1:N
    mEst = intrinsicMatrix'*(R*Mi(i,:)'+t);
    ei = [mEst(1)/mEst(3) mEst(2)/mEst(3)]-mi(i,:);
    eRes(i) = ei(1);
    eRes(N+i) = ei(2);
    e = e+norm(ei);
end




