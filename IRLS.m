function [rV, t] = IRLS(rVtNull, intrinsicMatrix, Mi, mi,N, tau)
rVtEst = rVtNull;
T = 0;
lambda = 0.001;
u = tau +1;
c = 4.685;


%% main loop
while T < N && u > tau
    %T
    %eRes = getRes(rVtEst, intrinsicMatrix,Mi,mi);
    J = computeJacobian(rVtEst, intrinsicMatrix,Mi);
    [e, eRes] = computeEnergi(rVtEst, intrinsicMatrix,Mi,mi);
    sigma = 1.48257968*mad(eRes,1);
    W = w(eRes/sigma,c);
    delta = -(J'*W*J+lambda*eye(6))^-1*(J'*W*eRes);
    [eNew,eResNew] = computeEnergi(rVtEst+delta,intrinsicMatrix,Mi,mi);
    if eNew > e
        lambda = 10*lambda;
    else
        lambda = lambda/10;
        rVtEst = rVtEst + delta;
    end
    u = norm(delta);
    T = T+1;
end
rV = rVtEst(1:3);
t = rVtEst(4:6);