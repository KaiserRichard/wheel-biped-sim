function out = derive_wbr(genDir)
% Sagittal Lagrangian: wheel, one equivalent leg of frozen length L, body. Writes gen_M, gen_V_grad, gen_G to genDir.
syms s th ph sd thd phd real
syms mw Iw r ml lc Il L mb Ib cx cz g real
q = [s; th; ph]; qd = [sd; thd; phd];

% Positions. Wheel axle at (s, r). Leg leans forward with +th. Body rotates forward with +ph about the hip.
hip = [s + L*sin(th); r + L*cos(th)];
legc = [s + lc*sin(th); r + lc*cos(th)];
bodc = hip + [cos(ph) sin(ph); -sin(ph) cos(ph)]*[cx; cz];

vel = @(x) jacobian(x, q)*qd;
vl = vel(legc); vb = vel(bodc);
T = (mw*sd^2 + Iw*(sd/r)^2 + ml*(vl.'*vl) + Il*thd^2 + mb*(vb.'*vb) + Ib*phd^2)/2;
V = g*(ml*legc(2) + mb*bodc(2));

M = simplify(hessian(T, qd));
% Velocity terms vanish at rest, so only M and the gravity gradient and its Jacobian are needed for A and B.
Vq = simplify(gradient(V, q));
Vqq = simplify(jacobian(Vq, q));
% Virtual work: wheel torque between leg and wheel, hip torque between body and leg.
syms Tw Tp real
Qdelta = Tw*(1/r)*[1;0;0] + Tw*[0;-1;0] + Tp*[0;1;-1];
G = jacobian(Qdelta, [Tw; Tp]);

args = {s, th, ph, mw, Iw, r, ml, lc, Il, L, mb, Ib, cx, cz, g};
matlabFunction(M, 'File', fullfile(genDir, 'gen_M'), 'Vars', args);
matlabFunction(Vq, 'File', fullfile(genDir, 'gen_Vq'), 'Vars', args);
matlabFunction(Vqq, 'File', fullfile(genDir, 'gen_Vqq'), 'Vars', args);
matlabFunction(G, 'File', fullfile(genDir, 'gen_G'), 'Vars', {r});
out = struct('M', char(M), 'Vq', char(Vq), 'Vqq', char(Vqq), 'G', char(G), 'T', char(T), 'V', char(V));
end
