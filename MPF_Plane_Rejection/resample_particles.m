function X = resample_particles(X, W)

N = size(X, 2);
Cum_sum = cumsum(W, 2);
rd = rand(1, N);
[~, idx] = histc(rd, Cum_sum);
X = X(:, idx + 1);
