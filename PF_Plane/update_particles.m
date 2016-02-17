function X = update_particles(Xstd_pos, X)

N = size(X, 2);

X(1:3,:) = X(1:3,:) + Xstd_pos * randn(3, N);
