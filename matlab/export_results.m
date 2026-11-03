function export_results(res, sym_txt, s, cross)
% Plain JSON and CSV, readable without MATLAB after the trial ends.
n = numel(s.L);
writematrix(s.L(:), fullfile(res, 'L.csv'));
writematrix(reshape(s.A, n, 36), fullfile(res, 'A_flat.csv'));
writematrix(reshape(s.B, n, 12), fullfile(res, 'B_flat.csv'));
writematrix(reshape(s.K, n, 12), fullfile(res, 'K_flat.csv'));
writematrix(s.coef, fullfile(res, 'K_poly_coef.csv'));
writematrix(s.eig_real, fullfile(res, 'closed_loop_eig_real.csv'));
writetext(fullfile(res, 'symbolic.json'), jsonencode(sym_txt));
summary = struct('L', s.L, 'th0', s.th0, 'Tp0', s.Tp0, 'fit_max_err', s.fit_max_err, 'worst_eig', s.worst_eig, 'worst_eig_fit', s.worst_eig_fit);
summary.A = permute(s.A, [2 3 1]);
summary.B = permute(s.B, [2 3 1]);
summary.K = permute(s.K, [2 3 1]);
summary.crosscheck = rmfield(cross, {'t', 'x_simulink', 'x_ode45'});
writetext(fullfile(res, 'summary.json'), jsonencode(summary));
for i = 1:numel(cross)
    writematrix([cross(i).t, cross(i).x_simulink, cross(i).x_ode45], fullfile(res, sprintf('response_L%.3f.csv', cross(i).L)));
end
end

function writetext(path, txt)
f = fopen(path, 'w');
fwrite(f, txt);
fclose(f);
end
