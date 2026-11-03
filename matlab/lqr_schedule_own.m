function s = lqr_schedule_own(p, w, n, deg)
% LQR over the leg range, cubic fit of each gain in L, closed-loop eigenvalues on and between grid points.
Ls = linspace(p.fivebar.leg_height_min, p.fivebar.leg_height_max, n);
Q = diag(w.Q); R = diag(w.R);
Ks = zeros(n, 2, 6); As = zeros(n, 6, 6); Bs = zeros(n, 6, 2); eigs_cl = zeros(n, 6); th0 = zeros(n, 1); Tp0 = zeros(n, 1);
for i = 1:n
    [A, B, th0(i), Tp0(i)] = linear_at(p, Ls(i));
    K = lqr(A, B, Q, R);
    Ks(i, :, :) = K; As(i, :, :) = A; Bs(i, :, :) = B;
    eigs_cl(i, :) = sort(real(eig(A - B*K))).';
end
coef = zeros(deg + 1, 12);
flat = reshape(Ks, n, 12);
fitv = zeros(n, 12);
for j = 1:12
    % Centred and scaled fit stays well conditioned; stored as plain polynomial coefficients in L.
    coef(:, j) = polyfit(Ls(:), flat(:, j), deg).';
    fitv(:, j) = polyval(coef(:, j), Ls(:));
end
worst_fit = -inf;
for L = linspace(Ls(1), Ls(end), 3*n)
    [A, B] = linear_at(p, L);
    K = reshape(arrayfun(@(j) polyval(coef(:, j), L), 1:12), 2, 6);
    worst_fit = max(worst_fit, max(real(eig(A - B*K))));
end
s = struct('L', Ls, 'K', Ks, 'A', As, 'B', Bs, 'eig_real', eigs_cl, 'th0', th0, 'Tp0', Tp0, 'coef', coef, ...
    'fit_max_err', max(abs(fitv(:) - flat(:))), 'worst_eig', max(eigs_cl(:)), 'worst_eig_fit', worst_fit);
end
