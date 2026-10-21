%% CMP01: paired RRT / RRT-star analysis for T2/O1
% Save as <repository>/analysis/analyze_T2_O1.m.
% Archive reference commit (not a per-run execution hash):
% dddf0799e85e61c08c5207f84a5c642b864a1825
clear; close all; clc;
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
resultsDir = fullfile(projectRoot, 'Results');
outDir = fullfile(scriptDir, 'outputs', 'CMP01_T2_O1');
if ~isfolder(outDir), mkdir(outDir); end

% Explicit manifest: columns are RRT and RRT-star, irrespective of run order.
seeds = [104729;130363;155921;196613;262147;314159;524287;654323;786433;999983];
runIds = [ ...
 "20261008_151048_V0_B_O1_T2", "20261008_151531_V0_B_O1_T2";
 "20261008_154820_V0_B_O1_T2", "20261008_153927_V0_B_O1_T2";
 "20261008_155319_V0_B_O1_T2", "20261008_155654_V0_B_O1_T2";
 "20261008_160751_V0_B_O1_T2", "20261008_161301_V0_B_O1_T2";
 "20261008_174714_V0_B_O1_T2", "20261008_184203_V0_B_O1_T2";
 "20261008_211012_V0_B_O1_T2", "20261008_211445_V0_B_O1_T2";
 "20261012_094238_V0_B_O1_T2", "20261012_094504_V0_B_O1_T2";
 "20261012_100843_V0_B_O1_T2", "20261012_100001_V0_B_O1_T2";
 "20261012_101544_V0_B_O1_T2", "20261012_102050_V0_B_O1_T2";
 "20261012_103922_V0_B_O1_T2", "20261012_103018_V0_B_O1_T2"];
algorithms = ["RRT", "RRT-star"];
prefixes = ["rrt", "rrt_star"];
n = numel(seeds);
lengthMM = nan(n,2); wallTime = nan(n,2);
histories = cell(n,1); records = struct([]);
baseline = []; baselinePath = [];

% Parameters which must be identical across the campaign.
commonKeys = ["rrt_max_iterations","rrt_extension_m","rrt_goal_bias", ...
 "rrt_bounds_min_x_m","rrt_bounds_max_x_m","rrt_bounds_min_y_m", ...
 "rrt_bounds_max_y_m","rrt_bounds_min_z_m","rrt_bounds_max_z_m", ...
 "segment_resolution_rad","segment_min_intervals","max_joint_step_rad", ...
 "max_rcm_error_allowed_m","max_tip_error_allowed_m", ...
 "min_clearance_allowed_m","min_obstacle_clearance_allowed_m", ...
 "ik_damping","ik_max_iterations","ik_position_tolerance_m", ...
 "ik_orientation_tolerance_rad","max_ik_attempts", ...
 "planning_profile_enabled","obstacle_enabled", ...
 "start_x_m","start_y_m","start_z_m","goal_x_m","goal_y_m","goal_z_m", ...
 "anatomy_world_x_m","anatomy_world_y_m","anatomy_world_z_m", ...
 "anatomy_world_qx","anatomy_world_qy","anatomy_world_qz","anatomy_world_qw", ...
 "obstacle_anatomy_x_m","obstacle_anatomy_y_m","obstacle_anatomy_z_m", ...
 "obstacle_anatomy_qx","obstacle_anatomy_qy","obstacle_anatomy_qz", ...
 "obstacle_anatomy_qw","obstacle_size_x_m","obstacle_size_y_m","obstacle_size_z_m"];
pathKeys = ["local_connector_step_m","local_connector_max_steps"];
jointNames = arrayfun(@(j) sprintf('q%d_rad',j),1:6,'UniformOutput',false);

for i = 1:n
    for a = 1:2
        id = runIds(i,a);
        stem = fullfile(resultsDir, char(id), 'V0_B_O1_T2');
        S = readSummary([stem '_summary.csv']);
        PS = readSummary([stem '_' char(prefixes(a)) '_summary.csv']);
        X = readtable([stem '_states.csv']);
        Q = readtable([stem '_' char(prefixes(a)) '_path.csv']);
        assert(textValue(S,'run_id') == id, 'Run ID mismatch: %s',id);
        assert(textValue(S,'target_name') == "T2", 'Wrong target: %s',id);
        assert(textValue(S,'execution_mode') == prefixes(a)+"_replay", 'Wrong mode: %s',id);
        assert(number(S,prefixes(a)+"_seed") == seeds(i), 'Wrong seed: %s',id);
        assert(number(S,'rrt_max_iterations') == 50, 'Wrong budget: %s',id);
        assert(textValue(S,'status') == "completed", 'Incomplete run: %s',id);
        assert(textValue(PS,'status') == "success" && number(PS,'restoration_passed') == 1, ...
            'Planning/restoration failed: %s',id);
        assert(textValue(S,'replay_policy') == "one_waypoint_per_simulation_step", 'Wrong replay policy');
        assert(textValue(S,'nominal_duration_s') == "not_applicable", 'Wrong duration interpretation');
        assert(textValue(S,'tau_definition') == "accepted_path_index_fraction", 'Wrong tau definition');
        assert(number(S,'obstacle_enabled') == 1 && number(S,'planning_profile_enabled') == 1, 'Wrong scenario/profile');
        values = arrayfun(@(k) number(S,k),commonKeys);
        pathValues = arrayfun(@(k) number(PS,k),pathKeys);
        if isempty(baseline)
            baseline = values; baselinePath = pathValues;
        else
            assert(all(abs(values-baseline)<1e-12), 'Campaign parameter changed: %s',id);
            assert(all(abs(pathValues-baselinePath)<1e-12), 'Connector parameter changed: %s',id);
        end
        m = height(X);
        assert(m > 1 && m == number(S,'accepted_samples') && m == height(Q) ...
            && m == number(PS,'returned_configurations'), 'Row count mismatch: %s',id);
        assert(isequal(X.sample,(0:m-1)') && isequal(Q.sample,(0:m-1)'), 'Sample indices mismatch');
        q = X{:,jointNames}; qp = Q{:,jointNames};
        assert(all(isfinite(q(:))) && all(isfinite(qp(:))), 'Nonfinite joints');
        assert(max(abs(q(:)-qp(:))) < 1e-12, 'Planned/replayed joint mismatch: %s',id);
        assert(max(max(abs(diff(q)))) <= number(S,'max_joint_step_rad')+1e-12, 'Joint step exceeded');
        assert(all(X.robot_wall_collision == 0) && all(X.robot_obstacle_collision == 0), 'Recorded collision');
        tip = X{:,{'tip_x_m','tip_y_m','tip_z_m'}};
        ref = X{:,{'target_x_m','target_y_m','target_z_m'}};
        assert(all(isfinite(tip(:))) && all(isfinite(ref(:))), 'Nonfinite positions');
        L = sum(sqrt(sum(diff(tip).^2,2)));
        LR = sum(sqrt(sum(diff(ref).^2,2)));
        near(L,number(S,'recorded_tip_path_length_m'),1e-12,'Tip path length');
        near(LR,number(S,'recorded_reference_path_length_m'),1e-12,'Reference path length');
        checks = [X.rcm_error_m,X.tip_error_m,X.clearance_m,X.shaft_obstacle_clearance_m];
        assert(all(isfinite(checks(:))), 'Nonfinite checks');
        maxima = max(checks(:,1:2),[],1); minima = min(checks(:,3:4),[],1);
        near(maxima(1),number(S,'max_rcm_error_m'),1e-12,'RCM maximum');
        near(maxima(2),number(S,'max_tip_error_m'),1e-12,'Tracking maximum');
        near(minima(1),number(S,'min_clearance_m'),1e-12,'Wall minimum');
        near(minima(2),number(S,'min_shaft_obstacle_clearance_m'),1e-12,'Obstacle minimum');
        assert(maxima(1)<=number(S,'max_rcm_error_allowed_m')+1e-12, 'RCM threshold exceeded');
        assert(maxima(2)<=number(S,'max_tip_error_allowed_m')+1e-12, 'Tracking threshold exceeded');
        assert(minima(1)>=number(S,'min_clearance_allowed_m')-1e-12, 'Wall threshold violated');
        assert(minima(2)>=number(S,'min_obstacle_clearance_allowed_m')-1e-12, 'Obstacle threshold violated');
        goal = [number(S,'goal_x_m'),number(S,'goal_y_m'),number(S,'goal_z_m')];
        finalError = norm(tip(end,:)-goal);
        near(finalError,number(S,'final_tip_to_goal_error_m'),1e-12,'Final goal error');
        lengthMM(i,a) = L*1000;
        wallTime(i,a) = number(S,'planning_wall_time_s');
        firstIteration = number(S,prefixes(a)+"_iterations");
        if a == 2
            H = readtable([stem '_rrt_star_cost_history.csv']);
            assert(height(H) == 1+number(S,'rrt_star_solution_improvements'), 'History count mismatch');
            assert(all(diff(H.best_cost_m)<0) && all(diff(H.iteration)>=0), 'Invalid cost history');
            near(H.best_cost_m(end),L,1e-12,'Final planner cost');
            firstIteration = number(S,'rrt_star_first_solution_iteration');
            assert(H.iteration(1)==firstIteration && H.iteration(end)<=50, 'History iteration mismatch');
            histories{i} = H;
        end
        r = struct('Seed',seeds(i),'Planner',algorithms(a),'RunID',id, ...
            'Status',textValue(S,'status'),'FirstSolutionIteration',firstIteration, ...
            'Iterations',number(S,prefixes(a)+"_iterations"), ...
            'PlanningTime_s',wallTime(i,a),'TipPathLength_mm',L*1000, ...
            'ReferencePathLength_mm',LR*1000,'Configurations',m, ...
            'MaxRCMError_mm',maxima(1)*1000,'MaxTrackingError_mm',maxima(2)*1000, ...
            'MinWallClearance_mm',minima(1)*1000,'MinObstacleClearance_mm',minima(2)*1000, ...
            'FinalGoalError_mm',finalError*1000);
        if isempty(records)
            records = r;
        else
            records(end+1) = r; %#ok<SAGROW>
        end
        fprintf('PASS  %-35s  %-8s  L=%.6f mm\n',char(id),char(algorithms(a)),L*1000);
    end
end

Runs = struct2table(records);
gain = 100*(lengthMM(:,1)-lengthMM(:,2))./lengthMM(:,1);
ratio = wallTime(:,2)./wallTime(:,1);
order = repmat("RRT then RRT-star",n,1);
for i=1:n
    ids = sort(runIds(i,:));
    if ids(1)==runIds(i,2), order(i)="RRT-star then RRT"; end
end
Pairs = table(seeds,runIds(:,1),runIds(:,2),order,lengthMM(:,1),lengthMM(:,2),gain, ...
    wallTime(:,1),wallTime(:,2),ratio, 'VariableNames', ...
    {'Seed','RRTRunID','RRTStarRunID','ExecutionOrder','RRTLength_mm','RRTStarLength_mm', ...
    'PairedLengthReduction_percent','RRTTime_s','RRTStarTime_s','TimeRatio_StarOverRRT'});
Summary = table(algorithms',repmat(n,2,1),mean(lengthMM,1)',median(lengthMM,1)', ...
    std(lengthMM,0,1)',min(lengthMM,[],1)',max(lengthMM,[],1)', ...
    mean(wallTime,1)',median(wallTime,1)',std(wallTime,0,1)', ...
    'VariableNames',{'Planner','SuccessfulRuns','MeanLength_mm','MedianLength_mm', ...
    'SampleSDLength_mm','MinLength_mm','MaxLength_mm','MeanPlanningTime_s', ...
    'MedianPlanningTime_s','SampleSDPlanningTime_s'});
writetable(Runs,fullfile(outDir,'runs.csv'));
writetable(Pairs,fullfile(outDir,'paired_comparison.csv'));
writetable(Summary,fullfile(outDir,'summary_statistics.csv'));
writetable(table(commonKeys',baseline','VariableNames',{'Parameter','Value'}), ...
    fullfile(outDir,'common_parameters.csv'));

% Figure 1: every seed remains visible, with a zero-based length axis.
f = figure('Color','w','Position',[100 100 1100 470]);
b = bar(lengthMM,'grouped');
b(1).FaceColor=[0.12 0.38 0.65]; b(2).FaceColor=[0.85 0.38 0.10];
xticks(1:n); xticklabels(string(seeds)); xtickangle(35);
xlabel('Random seed'); ylabel('Recorded tip path length (mm)');
title('T2/O1: path length by paired seed');
legend({'RRT','RRT*'},'Location','northoutside', ...
    'Orientation','horizontal','Box','off'); grid on; box off;
saveFigure(f,outDir,'path_lengths');

% Figure 2: measured planner wall time, not simulation playback duration.
f = figure('Color','w','Position',[100 100 1100 470]);
b = bar(wallTime,'grouped');
b(1).FaceColor=[0.12 0.38 0.65]; b(2).FaceColor=[0.85 0.38 0.10];
xticks(1:n); xticklabels(string(seeds)); xtickangle(35);
xlabel('Random seed'); ylabel('Planning wall time (s)');
title({'T2/O1: measured planning time (profiling enabled)', ...
    'RRT: first solution; RRT-star: full 50-iteration budget'});
legend({'RRT','RRT*'},'Location','northoutside', ...
    'Orientation','horizontal','Box','off'); grid on; box off;
saveFigure(f,outDir,'planning_times');

% Figure 3: no finite cost is drawn before the first solution.
f = figure('Color','w','Position',[100 100 1200 650]);
tl = tiledlayout(2,5,'TileSpacing','compact','Padding','compact');
for i=1:n
    nexttile(tl);
    H = histories{i};
    iterations = H.iteration; costs = H.best_cost_m*1000;
    if iterations(end)<50
        iterations(end+1)=50; costs(end+1)=costs(end);
    end
    stairs(iterations,costs,'LineWidth',1.5,'Color',[0.85 0.38 0.10]); hold on;
    plot(H.iteration,H.best_cost_m*1000,'o','MarkerSize',4,'Color',[0.85 0.38 0.10]);
    xlim([0 50]); ylim([3 8]); grid on; box off;
    title(sprintf('Seed %d',seeds(i)));
end
xlabel(tl,'Iteration','FontName','Arial','FontSize',16);
ylabel(tl,'Best recorded tip-path cost (mm)','FontName','Arial','FontSize',16);
title(tl,'RRT*: best retained cost (blank before first solution)', ...
    'FontName','Arial','FontSize',18,'FontWeight','normal');
saveFigure(f,outDir,'cost_histories');

fid = fopen(fullfile(outDir,'analysis_notes.txt'),'w');
assert(fid~=-1,'Cannot write analysis notes');
fprintf(fid,'CMP01, V0_B_O1_T2, 10 paired seeds, iteration budget 50.\n');
fprintf(fid,'Archive reference commit: dddf0799e85e61c08c5207f84a5c642b864a1825\n');
fprintf(fid,'This commit identifies the archived snapshot, not a verified execution hash for each run.\n');
fprintf(fid,'Only explicitly listed runs are included. T1/O1 and development runs are excluded.\n');
fprintf(fid,'RRT stops at first solution; RRT-star uses the full budget. Compute/time budgets are not equal.\n');
fprintf(fid,'Profiling enabled. Planning wall time includes geometric queries; none is subtracted.\n');
fprintf(fid,'One waypoint per simulation step is a kinematic replay policy, not physical motion timing.\n');
fprintf(fid,'CSV checks cover recorded states; they do not establish continuous collision freedom.\n');
fprintf(fid,'Path lengths are polylines of recorded actual tip positions, not reference positions.\n');
fprintf(fid,'Equal-cost tolerance for pair counts: 1e-9 mm. No global optimality claim.\n');
fprintf(fid,'RRT-star shorter: %d/10; equal within tolerance: %d/10.\n', ...
    sum(lengthMM(:,2)<lengthMM(:,1)-1e-9),sum(abs(diff(lengthMM,1,2))<=1e-9));
fprintf(fid,'Mean paired length reduction: %.6f percent.\n',mean(gain));
fprintf(fid,'Reduction of mean lengths (different statistic): %.6f percent.\n', ...
    100*(1-mean(lengthMM(:,2))/mean(lengthMM(:,1))));
fprintf(fid,'Actual execution order is recorded in paired_comparison.csv.\n');
fprintf(fid,'Seeds 196613 and 314159 used RRT first despite the planned alternating order.\n');
fprintf(fid,'Geometry mesh identity and source-code identity cannot be established from these CSVs alone.\n');
fclose(fid);
disp(Summary);
fprintf('\nAnalysis complete: 20 runs passed the implemented checks.\nOutputs: %s\n',outDir);

%% Local helpers
function S = readSummary(path)
    assert(isfile(path), 'Missing file: %s', path);

    fid = fopen(path, 'rt');
    assert(fid ~= -1, 'Cannot open summary: %s', path);
    closer = onCleanup(@() fclose(fid));

    C = textscan(fid, '%q%q', ...
        'Delimiter', ',', ...
        'HeaderLines', 1, ...
        'ReturnOnError', false);

    keys = string(C{1});
    values = string(C{2});

    assert(~isempty(keys) && numel(keys) == numel(values), ...
        'Expected parameter/value CSV: %s', path);

    assert(numel(unique(keys)) == numel(keys), ...
        'Duplicate summary keys: %s', path);

    S = containers.Map(cellstr(keys), cellstr(values));
end

function v = textValue(S,key)
    assert(isKey(S,char(key)),'Missing summary key: %s',key);
    v = string(S(char(key)));
end

function v = number(S,key)
    v = str2double(textValue(S,key));
    assert(isfinite(v),'Invalid numeric summary value: %s',key);
end

function near(a,b,tol,label)
    assert(isfinite(a) && isfinite(b) && abs(a-b)<=tol, ...
        '%s mismatch: %.16g versus %.16g',label,a,b);
end

function saveFigure(f,outDir,name)
    % Report typography, scoped to this figure (no global MATLAB defaults).
    % Keep the original canvas sizes and output filenames for Overleaf.
    if strcmp(name,'cost_histories')
        tickSize = 14; labelSize = 16; titleSize = 15;
    else
        tickSize = 18; labelSize = 20; titleSize = 18;
    end
    set(findall(f,'Type','text'),'FontName','Arial','FontSize',titleSize);
    axesList = findall(f,'Type','axes');
    for k = 1:numel(axesList)
        ax = axesList(k);
        set(ax,'FontName','Arial','FontSize',tickSize,'LineWidth',0.8);
        set(ax.XLabel,'FontSize',labelSize,'FontWeight','normal');
        set(ax.YLabel,'FontSize',labelSize,'FontWeight','normal');
        set(ax.Title,'FontSize',titleSize,'FontWeight','normal');
    end
    set(findall(f,'Type','legend'),'FontName','Arial','FontSize',16);
    drawnow;
    exportgraphics(f,fullfile(outDir,[name '.pdf']),'ContentType','vector');
    exportgraphics(f,fullfile(outDir,[name '.png']),'Resolution',300);
end
