function X = create_particles(D_Trgt, N)

X = zeros(6,N);
for i = 1:N
    X(1,i) = normrnd(D_Trgt(1,1), D_Trgt(2,1));
    X(2,i) = normrnd(D_Trgt(1,2), D_Trgt(2,2));
    X(3,i) = normrnd(D_Trgt(1,3), D_Trgt(2,3));
    X(4,i) = normrnd(D_Trgt(1,4), D_Trgt(2,4));
    X(5,i) = normrnd(D_Trgt(1,5), D_Trgt(2,5));
    X(6,i) = normrnd(D_Trgt(1,6), D_Trgt(2,6));
end
