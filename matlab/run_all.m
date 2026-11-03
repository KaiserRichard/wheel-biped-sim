function run_all()
% Runs derivation, LQR schedule, Simulink cross-check and export. Errors stop the run with a non-zero exit.
here = fileparts(mfilename('fullpath'));
gen = fullfile(here, 'generated');
res = fullfile(here, 'results');
if ~isfolder(gen), mkdir(gen); end
if ~isfolder(res), mkdir(res); end
addpath(here); addpath(gen);
try
    sym_txt = derive_wbr(gen);
    rehash;
    p = jsondecode(fileread(fullfile(here, 'inputs', 'params_design.json')));
    w = jsondecode(fileread(fullfile(here, 'inputs', 'weights.json')));
    sched = lqr_schedule_own(p, w, 15, 3);
    Ls = [p.fivebar.leg_height_min, 0.18, p.fivebar.leg_height_max];
    cross = simulink_crosscheck(p, w, Ls, deg2rad(2), 5);
    export_results(res, sym_txt, sched, cross);
    fprintf('run_all finished: worst eig %.4f, fitted %.4f, fit err %.5f\n', sched.worst_eig, sched.worst_eig_fit, sched.fit_max_err);
catch err
    fprintf(2, 'run_all FAILED: %s\n', getReport(err, 'extended', 'hyperlinks', 'off'));
    exit(1);
end
end
