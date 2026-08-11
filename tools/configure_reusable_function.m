% tools/configure_reusable_function.m
% Configure the missile model's code interface for reusable-function
% packaging so each generated missile_step() call operates on a
% per-instance RT_MODEL_missile_T pointer, no file-scope globals.
%
% Run once per fresh MATLAB session before rtwbuild.

function configure_reusable_function()
    model = 'missile';

    if ~bdIsLoaded(model)
        fprintf('[configure_reusable_function] Loading %s...\n', model);
        load_system(model);
    end

    fprintf('[configure_reusable_function] Setting Code Interface Packaging = Reusable function\n');

    set_param(model, 'SystemTargetFile', 'ert.tlc');
    set_param(model, 'CodeInterfacePackaging', 'Reusable function');
    set_param(model, 'GenerateAllocFcn', 'on');
    set_param(model, 'SupportContinuousTime', 'on');
    set_param(model, 'IncludeMdlTerminateFcn', 'on');
    set_param(model, 'GenCodeOnly', 'on');
    set_param(model, 'MatFileLogging', 'off');
    set_param(model, 'GenerateReport', 'on');

    % Solver: fixed-step ode4, matches autopilot / radar companion models
    set_param(model, 'SolverType', 'Fixed-step');
    set_param(model, 'Solver', 'ode4');
    set_param(model, 'FixedStep', 'CODEGEN_STEP');   % defined in model/missile_params.m

    save_system(model);
    fprintf('[configure_reusable_function] Saved. Ready for rtwbuild(''%s'').\n', model);
end
