function result = margins(A, B, K, C)
% Frequency-domain gain and phase margins of the assumed linearized loop.
% A, B and C are supplied by the caller. No actuator or sensor delay is included.
sys = ss(A, B, C, zeros(size(C, 1), size(B, 2)));
loop = sys * K;
result = struct();
for i = 1:size(B, 2)
    [gm, pm, wcg, wcp] = margin(loop(:, i));
    result(i).gain_margin = gm;
    result(i).phase_margin_deg = pm;
    result(i).gain_cross_rad_s = wcg;
    result(i).phase_cross_rad_s = wcp;
end
end
