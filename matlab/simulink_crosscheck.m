function r = simulink_crosscheck(p, w, Ls, tilt, tEnd)
% Closed loop x' = (A - B K) x from a small body tilt: script-built Simulink model against ode45.
Q = diag(w.Q); R = diag(w.R);
r = struct('L', {}, 'stop_time_reached', {}, 'final_time', {}, 'max_abs_diff', {}, 't', {}, 'x_simulink', {}, 'x_ode45', {});
mdl = 'wbr_linear_cl';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
cleanup = onCleanup(@() close_system(mdl, 0));
add_block('simulink/Continuous/State-Space', [mdl '/plant']);
add_block('simulink/Sources/Constant', [mdl '/zero'], 'Value', 'zeros(6,1)');
add_block('simulink/Sinks/Out1', [mdl '/x']);
add_line(mdl, 'zero/1', 'plant/1');
add_line(mdl, 'plant/1', 'x/1');
for i = 1:numel(Ls)
    [A, B] = linear_at(p, Ls(i));
    K = lqr(A, B, Q, R);
    Acl = A - B*K;
    x0 = zeros(6, 1); x0(5) = tilt;
    set_param([mdl '/plant'], 'A', mat2str(Acl, 17), 'B', 'zeros(6)', 'C', 'eye(6)', 'D', 'zeros(6)', 'X0', mat2str(x0));
    tOut = (0:0.002:tEnd).';
    out = sim(mdl, 'StopTime', num2str(tEnd), 'Solver', 'ode45', 'RelTol', '1e-8', 'AbsTol', '1e-10', ...
        'OutputOption', 'SpecifiedOutputTimes', 'OutputTimes', mat2str(tOut), 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
        'SaveFormat', 'Array', 'SaveTime', 'on', 'TimeSaveName', 'tout', 'ReturnWorkspaceOutputs', 'on');
    t = out.get('tout'); xs = out.get('yout');
    [~, xo] = ode45(@(~, x) Acl*x, tOut, x0, odeset('RelTol', 1e-8, 'AbsTol', 1e-10));
    n = min(size(xs, 1), size(xo, 1));
    r(i).L = Ls(i);
    r(i).final_time = t(end);
    r(i).stop_time_reached = abs(t(end) - tEnd) < 1e-9;
    r(i).max_abs_diff = max(abs(xs(1:n, :) - xo(1:n, :)), [], 'all');
    r(i).t = t;
    r(i).x_simulink = xs;
    r(i).x_ode45 = xo;
end
end
