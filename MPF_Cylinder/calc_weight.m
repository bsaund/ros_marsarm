function W = calc_weight(Xstd_tran, X0, X)
N = size(X,2);
W = zeros(1,N);

A = 1.0./(sqrt(2 * pi) * Xstd_tran);
B = - 0.5 / (Xstd_tran.^2);

for k = 1:N     
    W(k)=sum(A .* exp(B .* ((X0(1,:)-X(1,k)).^2+(X0(2,:)-X(2,k)).^2+(X0(3,:)-X(3,k)).^2+(X0(4,:)-X(4,k)).^2+(X0(5,:)-X(5,k)).^2)));
end

W = W / sum(W, 2);