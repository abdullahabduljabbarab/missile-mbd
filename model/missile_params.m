% model/missile_params.m
% Parameter script for missile guidance project (executable script)

% Metadata
TIMESTAMP = datestr(now,'yyyy-mm-dd HH:MM:SS');
MATLAB_VERSION = version;

% -- Guidance law parameters
% Navigation constant. Typical tactical range 3..5; higher N reduces
% miss distance for a maneuvering target but requires more latax.
N = 4;

% -- Missile envelope
% Lateral-acceleration limit (m/s^2). Representative short-range
% interceptor: 40g at sea level = ~392 m/s^2.
LATAX_MAX = 40 * 9.81;

% Initial missile speed at launch (m/s). Representative short-range
% interceptor burnout speed; kinematic 3-DOF, so this is treated as
% constant-magnitude velocity along the initial direction to target.
V_M0 = 500;

% -- Engagement termination
% Lethal miss radius (m). Intercept declared when |r_T - r_M| < R_LETHAL.
R_LETHAL = 25;

% Maximum engagement time (s). Timeout otherwise.
T_MAX = 60;

% -- Initial conditions for default test scenario
% Missile launch position (m, inertial frame).
R_M0 = [0; 0; 0];

% Target initial position and constant velocity for the smoke-sim
% scenario. Target flying laterally across the missile boresight.
R_T0 = [5000; 500; 1000];
V_T0 = [-100; 0; 0];

% -- Lead-collision-course initial missile velocity
% Real launchers point the missile at a predicted intercept point,
% not the target's current position. Approximates single-shot lead
% angle using constant-velocity target assumption:
%   1. Assume missile flies straight at V_M0
%   2. Estimate closing rate along LOS
%   3. Estimate time-to-go
%   4. Point missile at where target will be at t_go
% Prop-nav corrects the residual in flight; the lead angle removes
% the first-order geometry error that a v1 point-mass model cannot
% recover from with finite airframe latax.
tmp_r_rel        = R_T0 - R_M0;
tmp_range        = norm(tmp_r_rel);
tmp_r_hat        = tmp_r_rel / tmp_range;
tmp_V_close_los  = V_M0 - dot(V_T0, tmp_r_hat);      % missile closing rate along LOS
tmp_t_go_est     = tmp_range / max(tmp_V_close_los, 1); % guard against divide-by-zero
tmp_intercept_pt = R_T0 + V_T0 * tmp_t_go_est;
tmp_lead_dir     = (tmp_intercept_pt - R_M0) / norm(tmp_intercept_pt - R_M0);
V_M_INIT         = V_M0 * tmp_lead_dir;              % missile initial velocity vector, lead computed

fprintf('Lead-collision-course initial velocity: [%.1f %.1f %.1f] m/s (t_go_est %.2fs)\n', ...
    V_M_INIT(1), V_M_INIT(2), V_M_INIT(3), tmp_t_go_est);

clear tmp_r_rel tmp_range tmp_r_hat tmp_V_close_los tmp_t_go_est tmp_intercept_pt tmp_lead_dir

% -- Simulation and codegen settings
CODEGEN_STEP = 0.02;      % 50 Hz fixed step, matches autopilot / radar companion models
SIM_STOP_TIME = T_MAX;    % smoke sim stops at timeout unless intercept fires first

% -- Verification tolerances
% Regression check tolerances vs frozen baseline in ci_artifacts/
TOL_R = 0.5;              % missile position, m
TOL_V = 0.5;              % missile velocity, m/s
TOL_TERM = 0;             % termination flag must match exactly

% -- Convenience conversions
G_EARTH = 9.81;
LATAX_MAX_G = LATAX_MAX / G_EARTH;

% -- Logging
LOG_TO_WORKSPACE = true;
VERBOSE = true;

if VERBOSE
    fprintf('Loaded missile_params: N=%.1f, LATAX_MAX=%.1f m/s^2 (%.1f g), V_M0=%.1f m/s, R_LETHAL=%.1f m\n', ...
        N, LATAX_MAX, LATAX_MAX_G, V_M0, R_LETHAL);
end

MISSILE_METADATA.timestamp = TIMESTAMP;
MISSILE_METADATA.MATLAB_VERSION = MATLAB_VERSION;
assignin('base','MISSILE_METADATA',MISSILE_METADATA);
