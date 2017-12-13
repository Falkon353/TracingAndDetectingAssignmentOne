function W = w(e,c)

[N, M]= size(e);
W = zeros(N,N);
for i = 1:N
    if e(i) < c
        W(i,i) = (1-e(i)^2/c^2)^2;
    else
        W(i,i) = 0;
    end
end
