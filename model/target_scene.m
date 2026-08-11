% model/target_scene.m
% Reference target trajectory generator for smoke-sim and verification.
%
% Produces target position and velocity arrays over the sim horizon
% for a small set of canonical engagement geometries. Each scene is
% deterministic and drives the same baseline regression in
% ci_artifacts/simOut_missile_defaults.mat.

function scene = target_scene(scene_name, t)
% scene_name: one of 'lateral_crossing', 'head_on', 'tail_chase', 'maneuvering'
% t:          time vector (s), typically 0:CODEGEN_STEP:T_MAX
%
% scene.r_T: 3xN target position (m, inertial frame)
% scene.v_T: 3xN target velocity (m/s, inertial frame)
% scene.name: echo of scene_name

    N = numel(t);
    switch lower(scene_name)
        case 'lateral_crossing'
            % Target crossing missile boresight left-to-right at constant velocity.
            v_T = repmat([-100; 0; 0], 1, N);
            r_T = [5000; 500; 1000] + v_T .* t(:).';

        case 'head_on'
            % Target approaching the missile head-on at constant velocity.
            v_T = repmat([-200; 0; 0], 1, N);
            r_T = [8000; 0; 500] + v_T .* t(:).';

        case 'tail_chase'
            % Target moving away from missile in same direction as launch.
            v_T = repmat([150; 0; 0], 1, N);
            r_T = [1000; 0; 500] + v_T .* t(:).';

        case 'maneuvering'
            % Target executing a lateral 5g weave.
            g = 9.81;
            a_lat = 5 * g;
            omega = 0.5;   % rad/s
            v_T = zeros(3, N);
            r_T = zeros(3, N);
            r_T(:,1) = [5000; 0; 1000];
            v_T(:,1) = [-100; 0; 0];
            dt = t(2) - t(1);
            for k = 2:N
                a_k = [0; a_lat * cos(omega * t(k)); 0];
                v_T(:,k) = v_T(:,k-1) + a_k * dt;
                r_T(:,k) = r_T(:,k-1) + v_T(:,k-1) * dt;
            end

        otherwise
            error('target_scene:unknown_scene', 'Unknown scene: %s', scene_name);
    end

    scene.r_T = r_T;
    scene.v_T = v_T;
    scene.name = scene_name;
end
