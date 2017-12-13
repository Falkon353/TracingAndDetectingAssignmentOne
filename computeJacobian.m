function J = computeJacobian(rVtEst, intrinsicMatrix,Mi);
[N,M] = size(Mi);
J = zeros(2*N,6);
R = rotationVectorToMatrix(rVtEst(1:3))';
t = rVtEst(4:6)';
e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];

%% calculating the jacobian
for i = 1:N
    mEst = intrinsicMatrix'*(R*Mi(i,:)'+t);
    W = [1/mEst(3) 0 -mEst(1)/mEst(3)^2;
         0 1/mEst(3) -mEst(2)/mEst(3)^2];
    A = intrinsicMatrix';
    v = rVtEst(1:3);
    vSkew = [  0   -v(3)  v(2);
              v(3)   0   -v(1);
             -v(2)  v(1)   0  ];
    Rd1CrossProduct = cross(v',(eye(3)-R)*e1);
    Rd1CrossProductSkew = [      0             -Rd1CrossProduct(3)   Rd1CrossProduct(2);
                            Rd1CrossProduct(3)        0             -Rd1CrossProduct(1);
                           -Rd1CrossProduct(2)  Rd1CrossProduct(1)        0            ];
    Rd1 = (vSkew +Rd1CrossProductSkew)/norm(v)^2*R;
    Rd2CrossProduct = cross(v',(eye(3)-R)*e1);
    Rd2CrossProductSkew = [      0             -Rd2CrossProduct(3)   Rd2CrossProduct(2);
                            Rd2CrossProduct(3)        0             -Rd2CrossProduct(1);
                           -Rd2CrossProduct(2)  Rd2CrossProduct(1)        0            ];
    Rd2 = (vSkew +Rd2CrossProductSkew)/norm(v)^2*R;
    Rd3CrossProduct = cross(v',(eye(3)-R)*e1);
    Rd3CrossProductSkew = [      0             -Rd3CrossProduct(3)   Rd3CrossProduct(2);
                            Rd3CrossProduct(3)        0             -Rd3CrossProduct(1);
                           -Rd3CrossProduct(2)  Rd3CrossProduct(1)        0            ];
    Rd3 = (vSkew +Rd3CrossProductSkew)/norm(v)^2*R;
    RTd = [ Rd1*Mi(i,:)' Rd2*Mi(i,:)' Rd3*Mi(i,:)' eye(3)];
    Ji = W*A*RTd;
    J(2*i-1:2*i,:) = Ji;
end 
