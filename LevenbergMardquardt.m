function [rV, t] = LevenbergMardquardt(rVtNull, intrinsicMatrix, Mi, mi,N, tau)
%% initialize
rVtEst = rVtNull;
T = 0;
lambda = 0.001;
u = tau +1;
c = 4.685;


%% main loop
while T < N && u > tau
    %T
    
    J = computeJacobian(rVtEst, intrinsicMatrix,Mi);
    [e, eRes] = computeEnergi(rVtEst, intrinsicMatrix,Mi,mi);
    delta = -(J'*J+lambda*eye(6))^-1*(J'*eRes);
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





