% tools/run_model_tests_and_build.m
% CI entry point. Loads the model, runs the smoke sim + verification
% probes, runs the regression check, and attempts Embedded Coder
% codegen (best-effort on free GitHub-hosted runners; guaranteed on a
% licensed MATLAB Server).

function run_model_tests_and_build()
    fprintf('[ci] === missile-mbd CI ===\n');

    repo_root = fullfile(fileparts(mfilename('fullpath')), '..');
    addpath(genpath(repo_root));

    % Step 1: load parameters
    fprintf('[ci] Step 1: loading params\n');
    run(fullfile(repo_root, 'model', 'missile_params.m'));

    % Step 2: verification probe (T1)
    fprintf('[ci] Step 2: verify_prop_nav\n');
    try
        verify_prop_nav();
    catch err
        fprintf('[ci] verify_prop_nav failed: %s\n', err.message);
        rethrow(err);
    end

    % Step 3: regression check (T2)
    fprintf('[ci] Step 3: compare_sim\n');
    try
        compare_sim();
    catch err
        fprintf('[ci] compare_sim failed: %s\n', err.message);
        rethrow(err);
    end

    % Step 4: traceability report regeneration
    fprintf('[ci] Step 4: regenerating traceability report\n');
    try
        if exist('slreq.generateReport', 'file') == 2
            slreq.generateReport('missile', ...
                fullfile(repo_root, 'traceability_report.html'));
            fprintf('[ci] Traceability report regenerated\n');
        else
            fprintf('[ci] Simulink Requirements Toolbox not available; skipping report\n');
        end
    catch err
        fprintf('[ci] Traceability regen failed (non-fatal): %s\n', err.message);
    end

    % Step 5: Embedded Coder codegen (best-effort on free runners)
    fprintf('[ci] Step 5: rtwbuild (best-effort)\n');
    try
        configure_reusable_function();
        rtwbuild('missile');
        fprintf('[ci] Codegen succeeded\n');
    catch err
        fprintf('[ci] Codegen failed (non-fatal on unlicensed runner): %s\n', err.message);
    end

    fprintf('[ci] === done ===\n');
end
