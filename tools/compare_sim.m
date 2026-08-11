% tools/compare_sim.m
% T2 regression check: rerun the model against the frozen baseline
% scenario and assert every logged signal is within tolerance of the
% reference in ci_artifacts/simOut_missile_defaults.mat.

function compare_sim()
    fprintf('[compare_sim] Loading missile params...\n');
    run(fullfile(fileparts(mfilename('fullpath')), '..', 'model', 'missile_params.m'));

    baseline_path = fullfile(fileparts(mfilename('fullpath')), '..', ...
        'ci_artifacts', 'simOut_missile_defaults.mat');
    if ~exist(baseline_path, 'file')
        warning('compare_sim:no_baseline', ...
            'Baseline %s missing. Run the model once and save simOut to seed the baseline.', ...
            baseline_path);
        return
    end

    fprintf('[compare_sim] Loading baseline from %s...\n', baseline_path);
    baseline = load(baseline_path);

    fprintf('[compare_sim] Running fresh sim...\n');
    if ~exist('missile.slx', 'file') && ~exist('missile', 'file')
        warning('compare_sim:model_missing', ...
            'missile.slx not found. Author the Simulink model first, then rerun.');
        return
    end
    fresh = sim('missile');

    % Signals to compare (adapt names to actual model outports once authored)
    signals = {'r_M_out', 'v_M_out', 'term_flag_out'};
    tolerances = [TOL_R, TOL_V, TOL_TERM];

    fprintf('[compare_sim] Comparing %d signals against baseline...\n', numel(signals));
    for k = 1:numel(signals)
        name = signals{k};
        tol = tolerances(k);

        fresh_ts = fresh.get(name);
        base_ts = baseline.simOut.get(name);

        if isempty(fresh_ts) || isempty(base_ts)
            warning('compare_sim:missing_signal', 'Signal %s missing on one side; skipping', name);
            continue
        end

        drift = max(abs(fresh_ts.Data(:) - base_ts.Data(:)));
        fprintf('[compare_sim]  %s: max drift = %.4f (tol = %.4f)\n', name, drift, tol);
        assert(drift <= tol, ...
            'compare_sim:drift_exceeded', ...
            'Signal %s drifted %.4f beyond tolerance %.4f', name, drift, tol);
    end

    fprintf('[compare_sim] PASS\n');
end
