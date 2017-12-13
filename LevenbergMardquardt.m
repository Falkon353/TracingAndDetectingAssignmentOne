% function [R, t] = LevenbergMardquardt(Rnull,tNull, intriniscMatrix, M, m)
%% Constants
% RNull = % initial rotation matrix
% tNull = % initial translation vector
% M = [] % point you need to match
% m = [] % point to match to
M =  [0.020 0. 0.045];
m =  [1.3458e+03;1.1372e+03;1];
e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];

%% Estimates
% mEst = [] % estimate of m
% REst = [] % estimate of rotation matrix
% tEst = [] % estimate of translation vector
REst = R;
tEst = t';
RTEst = [REst tEst];

intrinsicMatrix = [2960.37845     0         0;
                       0      2960.37845    0;
                   1841.68855 1235.23369    1];
mEst = intrinsicMatrix'*(RTEst(:,1:3)*M'+RTEst(:,4));


%% calculating the jacobian
W = [1/mEst(3) 0 -mEst(1)/mEst(3)^2;
     0 1/mEst(3) -mEst(2)/mEst(3)^2];
A = intrinsicMatrix';
v = rotationMatrixToVector(REst);
vSkew = [  0   -v(3)  v(2);
          v(3)   0   -v(1);
         -v(2)  v(1)   0  ];
Rd1CrossProduct = cross(v',(eye(3)-REst)*e1);
Rd1CrossProductSkew = [      0             -Rd1CrossProduct(3)   Rd1CrossProduct(2);
                        Rd1CrossProduct(3)        0             -Rd1CrossProduct(1);
                       -Rd1CrossProduct(2)  Rd1CrossProduct(1)        0            ];
Rd1 = (vSkew +Rd1CrossProductSkew)/norm(v)^2*REst;
Rd2CrossProduct = cross(v',(eye(3)-REst)*e1);
Rd2CrossProductSkew = [      0             -Rd2CrossProduct(3)   Rd2CrossProduct(2);
                        Rd2CrossProduct(3)        0             -Rd2CrossProduct(1);
                       -Rd2CrossProduct(2)  Rd2CrossProduct(1)        0            ];
Rd2 = (vSkew +Rd2CrossProductSkew)/norm(v)^2*REst;
Rd3CrossProduct = cross(v',(eye(3)-REst)*e1);
Rd3CrossProductSkew = [      0             -Rd3CrossProduct(3)   Rd3CrossProduct(2);
                        Rd3CrossProduct(3)        0             -Rd3CrossProduct(1);
                       -Rd3CrossProduct(2)  Rd3CrossProduct(1)        0            ];
Rd3 = (vSkew +Rd3CrossProductSkew)/norm(v)^2*REst;
RTd = [ Rd1 Rd2 Rd3 eye(3)];
J_ana = W*A*RTd;
%% energy function
% Jacobian functor
J = @(x,h,F)(F(repmat(x,size(x'))+diag(h))-F(repmat(x,size(x'))))./h';
% Your function
f = @(x)norm(A*(x(:,1:3)*M'+x(:,4))-m)^2;
% Point at which to estimate it
%x = [1;1];
% Step to take on each dimension (has to be small enough for precision)
h = 1e-5*ones(size(RTEst));
% Compute the jacobian
J(RTEst,h,f)
%test = f(RTEst);



