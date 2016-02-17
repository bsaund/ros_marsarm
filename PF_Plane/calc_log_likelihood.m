function L = calc_log_likelihood(Xstd_dis, R, X, M)

N = size(X,2);
L = zeros(1,N);

A = -log(sqrt(2 * pi) * Xstd_dis);
B = - 0.5 / (Xstd_dis.^2);

for k = 1:N        
    D = ((M(1,:)+X(1,k).*M(2,:)+X(2,k).*M(3,:)+X(3,k))./sqrt(1+X(1,k)^2+X(2,k)^2))-R;
    temp=double(D<0);
    if uint8(temp*temp') == 0
        D2 = sum(D.^2);
        L(k) =  A + B * D2;
    else
        %L(k) = -inf;
        D2 = sum(D.^2);
        L(k) =  A + B * D2;
    end
end
