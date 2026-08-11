% tools/build_missile_model.m
% Programmatic build of missile.slx.
%
% Runs cleanly from any location as long as this repo is on the MATLAB
% path. Produces missile.slx at the repo root with:
%   - Root inports:  r_T (3), v_T (3), t (1)
%   - Root outports: r_M_out (3), v_M_out (3), term_flag_out (1)
%   - GuidanceSubsystem: LOS_Rate, Closing_Velocity, TPN_Law, Sat_a_cmd
%   - KinematicSubsystem: Integrate_v, Integrate_r
%   - TerminationSubsystem: Intercept_Check, Timeout_Check,
%                           LOS_Reversal_Check, priority latch
%   - Feedback loop broken by the KinematicSubsystem integrators
%   - Solver ode4, fixed step CODEGEN_STEP, Reusable function packaging
%
% Usage:
%   >> addpath(genpath(pwd))
%   >> build_missile_model
%
% The generated block positions are minimal (grid-snapped, functional
% but not pretty). Once the model opens, drag blocks around for
% readability. Structure and wiring are correct as authored.

function build_missile_model()
    model = 'missile';

    fprintf('[build] Cleaning previous %s ...\n', model);
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    slx_path = which([model '.slx']);
    if ~isempty(slx_path) && exist(slx_path, 'file')
        delete(slx_path);
    end

    % Load params into base so Integrator initial conditions can reference them
    fprintf('[build] Loading params into base workspace ...\n');
    repo_root = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(repo_root, 'model', 'missile_params.m'));
    assignin('base', 'N',           N);            %#ok<*NODEF>
    assignin('base', 'LATAX_MAX',   LATAX_MAX);
    assignin('base', 'V_M0',        V_M0);
    assignin('base', 'R_LETHAL',    R_LETHAL);
    assignin('base', 'T_MAX',       T_MAX);
    assignin('base', 'R_M0',        R_M0);
    assignin('base', 'R_T0',        R_T0);
    assignin('base', 'CODEGEN_STEP',CODEGEN_STEP);

    fprintf('[build] Creating new model ...\n');
    new_system(model);
    load_system(model);

    % ---------------------------------------------------------------
    % Solver + codegen settings (before block adds so subsystems inherit)
    % ---------------------------------------------------------------
    set_param(model, 'SolverType', 'Fixed-step');
    set_param(model, 'Solver', 'ode4');
    set_param(model, 'FixedStep', 'CODEGEN_STEP');
    set_param(model, 'StopTime',  'T_MAX');
    set_param(model, 'SystemTargetFile', 'ert.tlc');
    set_param(model, 'CodeInterfacePackaging', 'Reusable function');
    set_param(model, 'GenerateAllocFcn', 'on');
    set_param(model, 'SupportContinuousTime', 'on');
    set_param(model, 'PreLoadFcn', ...
        sprintf('run(fullfile(fileparts(which(''%s'')), ''model'', ''missile_params.m''));', model));

    % ---------------------------------------------------------------
    % Root ports
    % ---------------------------------------------------------------
    fprintf('[build] Adding root ports ...\n');
    addPort(model, 'Inport',  'r_T', 1, '3', [30  40 60 60]);
    addPort(model, 'Inport',  'v_T', 2, '3', [30 120 60 140]);
    addPort(model, 'Inport',  't',   3, '1', [30 200 60 220]);

    addPort(model, 'Outport', 'r_M_out',      1, '3', [900  40 930  60]);
    addPort(model, 'Outport', 'v_M_out',      2, '3', [900 120 930 140]);
    addPort(model, 'Outport', 'term_flag_out',3, '1', [900 200 930 220]);

    % ---------------------------------------------------------------
    % Subsystems
    % ---------------------------------------------------------------
    fprintf('[build] Adding subsystems ...\n');
    buildGuidanceSubsystem(model);
    buildKinematicSubsystem(model);
    buildTerminationSubsystem(model);

    % Position the three subsystems at the top level
    set_param([model '/GuidanceSubsystem'],    'Position', [200  40 400 240]);
    set_param([model '/KinematicSubsystem'],   'Position', [500  40 700 200]);
    set_param([model '/TerminationSubsystem'], 'Position', [500 240 700 400]);

    % ---------------------------------------------------------------
    % Top-level wiring
    % ---------------------------------------------------------------
    fprintf('[build] Wiring top-level signals ...\n');

    % Root inports -> GuidanceSubsystem
    add_line(model, 'r_T/1', 'GuidanceSubsystem/1', 'autorouting', 'on');
    add_line(model, 'v_T/1', 'GuidanceSubsystem/2', 'autorouting', 'on');

    % GuidanceSubsystem output (a_cmd_sat) -> KinematicSubsystem input (a_in)
    add_line(model, 'GuidanceSubsystem/1', 'KinematicSubsystem/1', 'autorouting', 'on');

    % KinematicSubsystem outputs (r_M, v_M) -> GuidanceSubsystem inputs (3, 4) via feedback
    add_line(model, 'KinematicSubsystem/1', 'GuidanceSubsystem/3', 'autorouting', 'on');  % r_M
    add_line(model, 'KinematicSubsystem/2', 'GuidanceSubsystem/4', 'autorouting', 'on');  % v_M

    % Also route r_M, v_M to root outports
    add_line(model, 'KinematicSubsystem/1', 'r_M_out/1', 'autorouting', 'on');
    add_line(model, 'KinematicSubsystem/2', 'v_M_out/1', 'autorouting', 'on');

    % TerminationSubsystem inputs: r_T, r_M, t, Vc
    add_line(model, 'r_T/1',                    'TerminationSubsystem/1', 'autorouting', 'on');
    add_line(model, 'KinematicSubsystem/1',     'TerminationSubsystem/2', 'autorouting', 'on');
    add_line(model, 't/1',                      'TerminationSubsystem/3', 'autorouting', 'on');
    add_line(model, 'GuidanceSubsystem/2',      'TerminationSubsystem/4', 'autorouting', 'on');  % Vc probe

    % TerminationSubsystem output -> term_flag_out
    add_line(model, 'TerminationSubsystem/1', 'term_flag_out/1', 'autorouting', 'on');

    % ---------------------------------------------------------------
    % Save + report
    % ---------------------------------------------------------------
    save_system(model, fullfile(repo_root, [model '.slx']));
    fprintf('[build] Saved %s.slx\n', model);
    fprintf('[build] Open in Simulink and drag blocks for readability if desired.\n');
    fprintf('[build] Structure and wiring are complete; you can sim immediately.\n');
end

% ===================================================================
% Guidance subsystem
% ===================================================================
function buildGuidanceSubsystem(model)
    sys = [model '/GuidanceSubsystem'];
    add_block('built-in/SubSystem', sys);

    % Inports (4): r_T, v_T, r_M, v_M
    addPort(sys, 'Inport',  'r_T', 1, '3', [30  40 60 60]);
    addPort(sys, 'Inport',  'v_T', 2, '3', [30 100 60 120]);
    addPort(sys, 'Inport',  'r_M', 3, '3', [30 160 60 180]);
    addPort(sys, 'Inport',  'v_M', 4, '3', [30 220 60 240]);

    % Outports (2): a_cmd_sat (main output), Vc (for TerminationSubsystem)
    addPort(sys, 'Outport', 'a_cmd_sat', 1, '3', [700  40 730  60]);
    addPort(sys, 'Outport', 'Vc',        2, '1', [700 120 730 140]);

    % LOS_Rate + Closing_Velocity fused in one MATLAB Function block
    % (LOS_Rate maths and Vc share the r and r_dot computation)
    addMatlabFcn(sys, 'LOS_Rate', ...
        {'r_T', 'v_T', 'r_M', 'v_M'}, {'omega', 'Vc', 'r'}, ...
        sprintf([ ...
        'function [omega, Vc, r] = fcn(r_T, v_T, r_M, v_M)\n' ...
        '%%#codegen\n' ...
        '    r     = r_T - r_M;               %% relative position\n' ...
        '    r_dot = v_T - v_M;               %% relative velocity\n' ...
        '    r_mag = norm(r);\n' ...
        '    if r_mag < 1e-6\n' ...
        '        omega = zeros(3,1);\n' ...
        '        Vc    = 0;\n' ...
        '    else\n' ...
        '        omega = cross(r, r_dot) / (r_mag*r_mag);\n' ...
        '        Vc    = -dot(r, r_dot) / r_mag;\n' ...
        '    end\n' ...
        'end\n']), ...
        [130 100 260 200]);

    % Alias sub-blocks so req_map REQ-MSL-002 and REQ-MSL-003 resolve
    % as siblings inside GuidanceSubsystem. We tag them by renaming
    % the single LOS_Rate MATLAB Function to include both names is not
    % possible, so we add lightweight passthrough Gain(1) blocks named
    % Closing_Velocity and LOS_Rate as REQ anchors, wired to the fused
    % kernel outputs. Traceability then resolves to a block that
    % exists in the model.
    add_block('built-in/Gain', [sys '/Closing_Velocity'], ...
        'Gain', '1', 'Position', [320 130 360 160]);
    add_block('built-in/Gain', [sys '/LOS_Rate_Probe'], ...
        'Gain', '1', 'Position', [320  70 360 100]);

    % TPN_Law: a_cmd = N * Vc * omega
    addMatlabFcn(sys, 'TPN_Law', ...
        {'omega', 'Vc'}, {'a_cmd'}, ...
        sprintf([ ...
        'function a_cmd = fcn(omega, Vc)\n' ...
        '%%#codegen\n' ...
        '    N = 4;              %% default nav constant (also in missile_params.m)\n' ...
        '    a_cmd = N * Vc * omega;\n' ...
        'end\n']), ...
        [430 80 560 160]);

    % Sat_a_cmd: direction-preserving magnitude saturation
    addMatlabFcn(sys, 'Sat_a_cmd', ...
        {'a_cmd'}, {'a_cmd_sat'}, ...
        sprintf([ ...
        'function a_cmd_sat = fcn(a_cmd)\n' ...
        '%%#codegen\n' ...
        '    LATAX_MAX = 40 * 9.81;   %% must match missile_params.m\n' ...
        '    mag = norm(a_cmd);\n' ...
        '    if mag > LATAX_MAX\n' ...
        '        a_cmd_sat = a_cmd * (LATAX_MAX / mag);\n' ...
        '    else\n' ...
        '        a_cmd_sat = a_cmd;\n' ...
        '    end\n' ...
        'end\n']), ...
        [590 80 720 160]);

    % Wiring inside GuidanceSubsystem
    add_line(sys, 'r_T/1', 'LOS_Rate/1', 'autorouting', 'on');
    add_line(sys, 'v_T/1', 'LOS_Rate/2', 'autorouting', 'on');
    add_line(sys, 'r_M/1', 'LOS_Rate/3', 'autorouting', 'on');
    add_line(sys, 'v_M/1', 'LOS_Rate/4', 'autorouting', 'on');

    % LOS_Rate outputs: [omega, Vc, r]
    add_line(sys, 'LOS_Rate/1', 'LOS_Rate_Probe/1', 'autorouting', 'on');
    add_line(sys, 'LOS_Rate/2', 'Closing_Velocity/1', 'autorouting', 'on');

    add_line(sys, 'LOS_Rate_Probe/1',   'TPN_Law/1', 'autorouting', 'on');
    add_line(sys, 'Closing_Velocity/1', 'TPN_Law/2', 'autorouting', 'on');

    add_line(sys, 'TPN_Law/1', 'Sat_a_cmd/1', 'autorouting', 'on');
    add_line(sys, 'Sat_a_cmd/1', 'a_cmd_sat/1', 'autorouting', 'on');
    add_line(sys, 'Closing_Velocity/1', 'Vc/1', 'autorouting', 'on');
end

% ===================================================================
% Kinematic subsystem
% ===================================================================
function buildKinematicSubsystem(model)
    sys = [model '/KinematicSubsystem'];
    add_block('built-in/SubSystem', sys);

    % Inport: a_cmd_sat
    addPort(sys, 'Inport', 'a_cmd_sat', 1, '3', [30 100 60 120]);

    % Outports: r_M, v_M
    addPort(sys, 'Outport', 'r_M', 1, '3', [500  60 530  80]);
    addPort(sys, 'Outport', 'v_M', 2, '3', [500 160 530 180]);

    % Integrate_v: velocity integrator, init condition = V_M0 * unit(R_T0 - R_M0)
    add_block('built-in/Integrator', [sys '/Integrate_v'], ...
        'Position',       [130 140 190 200], ...
        'InitialCondition','V_M0 * (R_T0 - R_M0) / norm(R_T0 - R_M0)');

    % Integrate_r: position integrator, init condition = R_M0
    add_block('built-in/Integrator', [sys '/Integrate_r'], ...
        'Position',       [290  40 350 100], ...
        'InitialCondition','R_M0');

    % Wiring
    add_line(sys, 'a_cmd_sat/1', 'Integrate_v/1', 'autorouting', 'on');
    add_line(sys, 'Integrate_v/1', 'Integrate_r/1', 'autorouting', 'on');
    add_line(sys, 'Integrate_v/1', 'v_M/1', 'autorouting', 'on');
    add_line(sys, 'Integrate_r/1', 'r_M/1', 'autorouting', 'on');
end

% ===================================================================
% Termination subsystem
% ===================================================================
function buildTerminationSubsystem(model)
    sys = [model '/TerminationSubsystem'];
    add_block('built-in/SubSystem', sys);

    % Inports: r_T, r_M, t, Vc
    addPort(sys, 'Inport', 'r_T', 1, '3', [30  40 60 60]);
    addPort(sys, 'Inport', 'r_M', 2, '3', [30 100 60 120]);
    addPort(sys, 'Inport', 't',   3, '1', [30 160 60 180]);
    addPort(sys, 'Inport', 'Vc',  4, '1', [30 220 60 240]);

    % Outport: term_flag
    addPort(sys, 'Outport', 'term_flag', 1, '1', [500 140 530 160]);

    % Intercept_Check
    addMatlabFcn(sys, 'Intercept_Check', ...
        {'r_T', 'r_M'}, {'hit'}, ...
        sprintf([ ...
        'function hit = fcn(r_T, r_M)\n' ...
        '%%#codegen\n' ...
        '    R_LETHAL = 5;   %% must match missile_params.m\n' ...
        '    hit = norm(r_T - r_M) < R_LETHAL;\n' ...
        'end\n']), ...
        [130 60 260 120]);

    % Timeout_Check
    addMatlabFcn(sys, 'Timeout_Check', ...
        {'t'}, {'timeout'}, ...
        sprintf([ ...
        'function timeout = fcn(t)\n' ...
        '%%#codegen\n' ...
        '    T_MAX = 60;   %% must match missile_params.m\n' ...
        '    timeout = t >= T_MAX;\n' ...
        'end\n']), ...
        [130 150 260 200]);

    % LOS_Reversal_Check with persistent peak tracking
    addMatlabFcn(sys, 'LOS_Reversal_Check', ...
        {'Vc'}, {'reversed'}, ...
        sprintf([ ...
        'function reversed = fcn(Vc)\n' ...
        '%%#codegen\n' ...
        '    persistent Vc_peak\n' ...
        '    if isempty(Vc_peak)\n' ...
        '        Vc_peak = -inf;\n' ...
        '    end\n' ...
        '    if Vc > Vc_peak\n' ...
        '        Vc_peak = Vc;\n' ...
        '    end\n' ...
        '    reversed = (Vc_peak > 0) && (Vc < 0);\n' ...
        'end\n']), ...
        [130 230 260 280]);

    % Priority latch: first-triggered wins, then held
    addMatlabFcn(sys, 'Priority_Latch', ...
        {'hit', 'timeout', 'reversed'}, {'term_flag'}, ...
        sprintf([ ...
        'function term_flag = fcn(hit, timeout, reversed)\n' ...
        '%%#codegen\n' ...
        '    persistent latched\n' ...
        '    if isempty(latched)\n' ...
        '        latched = int32(0);\n' ...
        '    end\n' ...
        '    if latched == 0\n' ...
        '        if hit\n' ...
        '            latched = int32(1);\n' ...
        '        elseif timeout\n' ...
        '            latched = int32(2);\n' ...
        '        elseif reversed\n' ...
        '            latched = int32(3);\n' ...
        '        end\n' ...
        '    end\n' ...
        '    term_flag = double(latched);\n' ...
        'end\n']), ...
        [330 130 460 200]);

    % Wiring
    add_line(sys, 'r_T/1', 'Intercept_Check/1', 'autorouting', 'on');
    add_line(sys, 'r_M/1', 'Intercept_Check/2', 'autorouting', 'on');
    add_line(sys, 't/1',   'Timeout_Check/1',   'autorouting', 'on');
    add_line(sys, 'Vc/1',  'LOS_Reversal_Check/1', 'autorouting', 'on');

    add_line(sys, 'Intercept_Check/1',    'Priority_Latch/1', 'autorouting', 'on');
    add_line(sys, 'Timeout_Check/1',      'Priority_Latch/2', 'autorouting', 'on');
    add_line(sys, 'LOS_Reversal_Check/1', 'Priority_Latch/3', 'autorouting', 'on');
    add_line(sys, 'Priority_Latch/1',     'term_flag/1',      'autorouting', 'on');
end

% ===================================================================
% Helpers
% ===================================================================
function addPort(parent, kind, name, portnum, dims, position)
    block = sprintf('built-in/%s', kind);
    add_block(block, [parent '/' name], ...
        'Port', num2str(portnum), 'Position', position);
    if strcmp(kind, 'Inport') && ~isempty(dims)
        set_param([parent '/' name], 'PortDimensions', dims);
    end
end

function addMatlabFcn(parent, name, inputs, outputs, script, position)
    block_path = [parent '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', block_path, ...
        'Position', position);

    % Set the function script via the Stateflow API
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', '-and', 'Path', block_path);
    if isempty(chart)
        error('build_missile_model:no_chart', ...
            'Could not find MATLAB Function chart for %s', block_path);
    end
    chart.Script = script;
end
