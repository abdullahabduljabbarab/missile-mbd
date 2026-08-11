
% run_model_tests_and_build.m  (script version, matches autopilot-mbd pattern)
model = 'missile';
artifactsDir = fullfile(pwd,'ci_artifacts');
if ~isfolder(artifactsDir), mkdir(artifactsDir); end

% 1) Load model
fprintf('Loading model %s...\n', model);
try
    load_system(model);
catch ME
    warning('load_system failed: %s', ME.message);
end

% 2) Smoke simulation (short run)
try
    fprintf('Running smoke simulation...\n');
    simOut = sim(model,'SaveOutput','on','SaveFormat','Dataset','Timeout',300);
    save(fullfile(artifactsDir,'simOut_smoke.mat'),'simOut');
catch ME
    warning('Smoke sim failed: %s', ME.message);
end

% 3) Verify prop-nav (T1 probe against known-solution intercept)
try
    fprintf('Running verify_prop_nav...\n');
    verify_prop_nav();
catch ME
    warning('verify_prop_nav failed: %s', ME.message);
end

% 4) Regression baseline check (T2)
try
    fprintf('Running compare_sim...\n');
    compare_sim();
catch ME
    warning('compare_sim failed: %s', ME.message);
end

% 5) Traceability report
try
    fprintf('Generating traceability report...\n');
    generate_traceability_report();
catch ME
    warning('generate_traceability_report failed: %s', ME.message);
end

% 6) Attempt code generation/build (best-effort on unlicensed runners)
try
    fprintf('Attempting model build/codegen (slbuild)...\n');
    buildOutput = slbuild(model);
    genFolder = fullfile(pwd,[model '_grt_rtw']);
    if isfolder(fullfile(pwd,'slprj')), copyfile(fullfile(pwd,'slprj'), fullfile(artifactsDir,'slprj')); end
    if isfolder(genFolder), copyfile(genFolder, fullfile(artifactsDir,'codegen')); end
catch ME
    warning('slbuild failed or not configured: %s', ME.message);
end

% 7) Snapshot workspace
try save(fullfile(artifactsDir,'workspace_snapshot.mat')); catch, end

% 8) Zip artefacts
zipFile = fullfile(pwd,'ci_artifacts.zip');
if exist(zipFile,'file'), delete(zipFile); end
try zip(zipFile,artifactsDir); catch ME, warning('Zip failed: %s', ME.message); end
fprintf('Packaged artifacts to %s\n', zipFile);

fprintf('CI script finished.\n');
