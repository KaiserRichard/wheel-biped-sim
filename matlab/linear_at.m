function [A, B, th0, Tp0] = linear_at(p, L)
% A, B about the rest point at leg length L with the body at phi = 0.
leg = leg_lump(p, L);
c = {2*p.wheel.mass, 2*p.wheel.inertia_axial, p.wheel.radius, leg.mass, leg.lc, leg.inertia, L, ...
    p.body.mass, p.body.inertia_diag(2), p.body.com_offset_xz(1), p.body.com_offset_xz(2), p.gravity};
% Adding the theta and phi rows cancels the hip torque.
f = @(th) [0 1 1]*gen_Vq(0, th, 0, c{:});
th0 = fzero(f, [-0.8 0.8]);
Vq = gen_Vq(0, th0, 0, c{:});
Tp0 = Vq(2);
M = gen_M(0, th0, 0, c{:});
K = gen_Vqq(0, th0, 0, c{:});
G = gen_G(p.wheel.radius);
pos = [1 3 5]; vel = [2 4 6];
A = zeros(6); B = zeros(6, 2);
A(pos, vel) = eye(3);
A(vel, pos) = -M\K;
B(vel, :) = M\G;
end
