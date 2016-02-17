function X = create_particles(D_Trgt, N)

X = zeros(5,N);
for i = 1:N
    X(1,i) = D_Trgt(1,1) + D_Trgt(2,1) * randn;
    X(2,i) = D_Trgt(1,2) + D_Trgt(2,2) * randn;
    X(3,i) = D_Trgt(1,3) + D_Trgt(2,3) * randn;
    X(4,i) = D_Trgt(1,4) + D_Trgt(2,4) * randn;
    X(5,i) = D_Trgt(1,5) + D_Trgt(2,5) * randn;
end

