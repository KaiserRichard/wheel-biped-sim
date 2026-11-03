function leg = leg_lump(p, L)
% Both legs lumped into one equivalent body at leg length L, with the leg straight below the hips.
% Each rod is uniform. Distances are measured up the leg from the wheel axle. Returns mass, lc and Iy about the COM.
fb = p.fivebar;
d = fb.hip_spacing/2; a = fb.upper_length; b = fb.lower_length;
% Knee from intersecting the circles round the hip (d, L) and the foot (0, 0), outer solution.
c = hypot(d, L);
h = (b^2 - a^2 + c^2)/(2*c);
k = sqrt(b^2 - h^2);
u = [d, L]/c;
n = [u(2), -u(1)];
knee = h*u + k*n;
hip = [d, L];
% One side: upper rod (hip to knee), lower rod (knee to foot).
rods = {hip, knee, fb.upper_mass, a; knee, [0 0], fb.lower_mass, b};
m = 0; mz = 0;
for i = 1:2
    mid = (rods{i,1} + rods{i,2})/2;
    m = m + rods{i,3};
    mz = mz + rods{i,3}*mid(2);
end
z = mz/m;
I = 0;
for i = 1:2
    mid = (rods{i,1} + rods{i,2})/2;
    I = I + rods{i,3}*rods{i,4}^2/12 + rods{i,3}*(mid(1)^2 + (mid(2) - z)^2);
end
% Two sides per leg, two legs.
leg.mass = 4*m;
leg.lc = z;
leg.inertia = 4*I;
end
