function [H, info] = hip_mode_model(c, ds, delay_ms, dL, dr)
%HIP_MODE_MODEL Differential five-bar mode with exact ZOH and motor torque FIFO.
% x = [L_left-L_right; alpha_left-alpha_right; their time derivatives].
% Numerical reduction inputs are in params_design.json; see README.md.
% ASSUMED: inherited ks=30, kL=800, kr=40, dt=0.002 s, frozen snapshots.
% Torque/speed saturation is NOT linearized. Estimated rates have no filter.
% No Control System or Robust Control Toolbox required.
if nargin < 4, dL = 50; end
if nargin < 5, dr = 5; end
dt = 0.002;
n = round(delay_ms/(1000*dt));
assert(n >= 0 && abs(delay_ms-n*1000*dt) < 1e-10, ...
    'The simulator FIFO supports integer control ticks only.');
A = c.A; B = c.B;
K = [c.Kp, dL*c.Kv_height + dr*c.Kv_roll + ds*c.Kv_sync];
% The exponential of the augmented continuous system gives exact ZOH.
E = expm([A, B; zeros(4,8)]*dt);
Ad = E(1:4,1:4); Bd = E(1:4,5:8);
if n == 0
    H = Ad + Bd*K;
else
    % State = [x_k; u_(k-1); ...; u_(k-n)].
    H = zeros(4+4*n);
    H(1:4,1:4) = Ad;
    H(1:4,end-3:end) = Bd;
    H(5:8,1:4) = K;
    if n > 1, H(9:end,5:end-4) = eye(4*(n-1)); end
end
if nargout > 1
    lambda = eig(H);
    [radius, j] = max(abs(lambda));
    info = struct('rho',radius,'stable',radius < 1, ...
        'dominant_real',real(lambda(j)),'dominant_imag',imag(lambda(j)), ...
        'dominant_Hz',abs(angle(lambda(j)))/(2*pi*dt), ...
        'dt_s',dt,'delay_ticks',n,'state_count',size(H,1));
end
end
