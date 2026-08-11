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
R_LETHAL = 5;

% Maximum engagement time (s). Timeout otherwise.
T_MAX = 60;

% -- Initial conditions for default test scenario
% Missile launch position (m, inertial frame).
R_M0 = [0; 0; 0];

% Target initial position and constant velocity for the smoke-sim
% scenario. Target flying laterally across the missile boresight.
R_T0 = [5000; 500; 1000];
V_T0 = [-100; 0; 0];

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
