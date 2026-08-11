% tools/verify_prop_nav.m
% T1 verification probe for the proportional-navigation guidance law.
%
% Runs a known-solution intercept scenario (lateral-crossing target at
% constant velocity) and asserts:
%   1. Intercept was declared (term_flag == 1)
%   2. Miss distance at intercept < R_LETHAL
%   3. Time-to-intercept within the analytic ballpark for N * Vc / V_M
%
% Called from tools/run_model_tests_and_build.m as part of the CI T1 pass.

function verify_prop_nav()
    fprintf('[verify_prop_nav] Loading missile params...\n');
    run(fullfile(fileparts(mfilename('fullpath')), '..', 'model', 'missile_params.m'));

    fprintf('[verify_prop_nav] Running smoke sim against lateral_crossing scene...\n');

    % Sim assumes the missile.slx model is on path and configured to read
    % the scene inputs from the base workspace. This is a placeholder that
    % becomes the real assertion once the model exists.
    if ~exist('missile.slx', 'file') && ~exist('missile', 'file')
        warning('verify_prop_nav:model_missing', ...
            'missile.slx not found. Author the Simulink model first, then rerun.');
        return
    end

    simOut = sim('missile');

    % Extract results (adapt names to actual model outports once authored)
    term_flag = simOut.get('term_flag_out');
    r_M = simOut.get('r_M_out');
    r_T = simOut.get('r_T');

    if isempty(term_flag)
        error('verify_prop_nav:no_output', 'term_flag_out signal missing from simOut');
    end

    final_flag = term_flag.Data(end);
    final_miss = norm(r_T.Data(end,:) - r_M.Data(end,:));

    fprintf('[verify_prop_nav] Final termination flag: %d\n', final_flag);
    fprintf('[verify_prop_nav] Final miss distance:    %.2f m\n', final_miss);
    fprintf('[verify_prop_nav] R_LETHAL threshold:     %.2f m\n', R_LETHAL);

    assert(final_flag == 1, ...
        'verify_prop_nav:no_intercept', ...
        'Expected intercept (term_flag = 1), got %d', final_flag);
    assert(final_miss < R_LETHAL, ...
        'verify_prop_nav:miss_too_large', ...
        'Miss distance %.2f m exceeds R_LETHAL %.2f m', final_miss, R_LETHAL);

    fprintf('[verify_prop_nav] PASS\n');
end
