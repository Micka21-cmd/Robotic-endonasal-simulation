%% CMP02: paired RRT / RRT-star analysis for T1/O2
% Save as <repository>/analysis/analyze_T1_O2.m.
% Pre-campaign reference commit (not a per-run execution hash):
% 59da78e2cc56827ecbad87abb7c739c986bd496c
clear; close all; clc;
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
resultsDir = fullfile(projectRoot, 'Results');
outDir = fullfile(scriptDir, 'outputs', 'CMP02_T1_O2');
if ~isfolder(outDir), mkdir(outDir); end

% Explicit manifest: columns are RRT and RRT-star, irrespective of run order.
seeds = [104729;130363;155921;196613;262147;314159;524287;654323;786433;999983];
runIds = [ ...
 "20261014_111141_V0_B_O2_T1", "20261014_111343_V0_B_O2_T1";
 "20261014_113725_V0_B_O2_T1", "20261014_112925_V0_B_O2_T1";
 "20261014_140303_V0_B_O2_T1", "20261014_140613_V0_B_O2_T1";
 "20261014_145759_V0_B_O2_T1", "20261014_144827_V0_B_O2_T1";
 "20261014_185327_V0_B_O2_T1", "20261014_185535_V0_B_O2_T1";
 "20261014_191350_V0_B_O2_T1", "20261014_190442_V0_B_O2_T1";
 "20261015_160855_V0_B_O2_T1", "20261015_161104_V0_B_O2_T1";
 "20261015_162834_V0_B_O2_T1", "20261015_162050_V0_B_O2_T1";
 "20261015_164702_V0_B_O2_T1", "20261015_165150_V0_B_O2_T1";
 "20261015_184838_V0_B_O2_T1", "20261015_183726_V0_B_O2_T1"];
algorithms = ["RRT", "RRT-star"];
prefixes = ["rrt", "rrt_star"];
n = numel(seeds);
lengthMM = nan(n,2); wallTime = nan(n,2); success = false(n,2);
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
        stem = fullfile(resultsDir, char(id), 'V0_B_O2_T1');
        S = readSummary([stem '_summary.csv']);
        PS = readSummary([stem '_' char(prefixes(a)) '_summary.csv']);
        X = readtable([stem '_states.csv']);
        Q = readtable([stem '_' char(prefixes(a)) '_path.csv']);
        assert(textValue(S,'run_id') == id, 'Run ID mismatch: %s',id);
        assert(textValue(S,'target_name') == "T1", 'Wrong target: %s',id);
        assert(textValue(S,'execution_mode') == prefixes(a)+"_replay", 'Wrong mode: %s',id);
        assert(number(S,prefixes(a)+"_seed") == seeds(i), 'Wrong seed: %s',id);
        assert(number(S,'rrt_max_iterations') == 50, 'Wrong budget: %s',id);
        status = textValue(S,'status');
        success(i,a) = status == "completed";
        assert(success(i,a) || status == "failed", 'Unexpected run status: %s',id);
        assert(number(PS,'restoration_passed') == 1, 'Restoration failed: %s',id);
        assert(textValue(S,'obstacle_path') == "/Anatomy_V0/Obstacle_O2", 'Wrong obstacle');
        assert(textValue(S,'run_label') == "V0_B_O2_T1", 'Wrong scenario');
        assert(textValue(PS,'run_id') == id, 'Path summary ID mismatch');
        profileKeys = string(keys(S));
        profileKeys = profileKeys(endsWith(profileKeys,'_lua_errors'));
        assert(all(arrayfun(@(k) number(S,k)==0,profileKeys)), 'Profile Lua error');
        if a == 2
            assert(number(S,'rrt_star_iterations') == 50, 'Incomplete RRT-star budget');
            near(number(S,'rrt_star_radius_max_m'),0.0015,1e-12,'Star radius');
            near(number(S,'rrt_star_gamma_m'),0.003,1e-12,'Star gamma');
            near(number(S,'rrt_star_neighbour_dimension'),3,0,'Star dimension');
        end
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
        wallTime(i,a) = number(S,'planning_wall_time_s');
        assert(wallTime(i,a)>=0, 'Negative planning time');
        L=NaN; LR=NaN; maxima=[NaN NaN]; minima=[NaN NaN];
        finalError=NaN; firstIteration=NaN;
        if ~success(i,a)
            assert(textValue(PS,'status') == "failed", 'Failure status mismatch');
            assert(m==0 && height(Q)==0 && number(S,'accepted_samples')==0 ...
                && number(PS,'returned_configurations')==0, 'Failed run contains a path');
            assert(number(S,prefixes(a)+"_iterations")==50, 'Failure before budget');
            assert(contains(textValue(S,'reason'),"iteration budget exhausted"), 'Unexpected failure reason');
            assert(number(PS,'replay_started')==0, 'Replay started after planning failure');
            if a==2
                H=readtable([stem '_rrt_star_cost_history.csv']);
                assert(height(H)==0 && number(S,'rrt_star_solution_improvements')==0, 'Failed run has cost history');
                histories{i}=H;
            end
        else
        assert(textValue(PS,'status') == "success", 'Planning status mismatch');
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
        assert(firstIteration>=1 && firstIteration<=50, 'Invalid first solution iteration');
        assert(finalError <= number(S,'max_tip_error_allowed_m')+1e-12, 'Final goal tolerance exceeded');
        end
        r = struct('Seed',seeds(i),'Planner',algorithms(a),'RunID',id, ...
            'Status',textValue(S,'status'),'Reason',textValue(S,'reason'),'FirstSolutionIteration',firstIteration, ...
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
        fprintf('CHECKED  %-35s  %-8s  %-9s  L=%.6f mm\n',char(id),char(algorithms(a)),char(status),L*1000);
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
Summary = table(algorithms',repmat(n,2,1),sum(success,1)',100*mean(success,1)',mean(lengthMM,1,'omitnan')',median(lengthMM,1,'omitnan')', ...
    std(lengthMM,0,1,'omitnan')',min(lengthMM,[],1,'omitnan')',max(lengthMM,[],1,'omitnan')', ...
    mean(wallTime,1)',median(wallTime,1)',std(wallTime,0,1)', ...
    'VariableNames',{'Planner','AttemptedRuns','SuccessfulRuns','SuccessRate_percent','MeanLength_mm','MedianLength_mm', ...
    'SampleSDLength_mm','MinLength_mm','MaxLength_mm','MeanPlanningTimeAllRuns_s', ...
    'MedianPlanningTimeAllRuns_s','SampleSDPlanningTimeAllRuns_s'});
writetable(Runs,fullfile(outDir,'runs.csv'));
Pairs.RRTSuccess = success(:,1); Pairs.RRTStarSuccess = success(:,2);
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
title({'T1/O2: recorded tip path length by paired seed', 'Missing bars: no solution within 50 iterations'});
for k=find(~any(success,2))'
    text(k,0.15,'Failed','HorizontalAlignment','center');
end
legend({'RRT','RRT*'},'Location','northoutside', ...
    'Orientation','horizontal','Box','off'); grid on; box off;
saveFigure(f,outDir,'path_lengths');

% Figure 2: measured planner wall time, not simulation playback duration.
f = figure('Color','w','Position',[100 100 1100 470]);
b = bar(wallTime,'grouped');
b(1).FaceColor=[0.12 0.38 0.65]; b(2).FaceColor=[0.85 0.38 0.10];
xticks(1:n); xticklabels(string(seeds)); xtickangle(35);
xlabel('Random seed'); ylabel('Planning wall time (s)');
title({'T1/O2: measured planning time (profiling enabled)', ...
    'RRT: first solution or budget exhausted; RRT-star: full 50 iterations'});
legend({'RRT','RRT*'},'Location','northoutside', ...
    'Orientation','horizontal','Box','off'); grid on; box off;
saveFigure(f,outDir,'planning_times');

% Figure 3: no finite cost is drawn before the first solution.
f = figure('Color','w','Position',[100 100 1200 650]);
tl = tiledlayout(2,5,'TileSpacing','compact','Padding','compact');
for i=1:n
    nexttile(tl);
    H = histories{i};
    if isempty(H)
        xlim([0 50]); ylim([3 4.5]); grid on; box off;
        text(25,3.75,'No solution','HorizontalAlignment','center');
        title(sprintf('Seed %d',seeds(i)));
        continue
    end
    iterations = H.iteration; costs = H.best_cost_m*1000;
    if iterations(end)<50
        iterations(end+1)=50; costs(end+1)=costs(end);
    end
    stairs(iterations,costs,'LineWidth',1.5,'Color',[0.85 0.38 0.10]); hold on;
    plot(H.iteration,H.best_cost_m*1000,'o','MarkerSize',4,'Color',[0.85 0.38 0.10]);
    xlim([0 50]); ylim([3 4.5]); grid on; box off;
    title(sprintf('Seed %d',seeds(i)));
end
xlabel(tl,'Iteration','FontName','Arial','FontSize',16);
ylabel(tl,'Best recorded tip-path cost (mm)','FontName','Arial','FontSize',16);
title(tl,'RRT*: best retained cost (blank before first solution)', ...
    'FontName','Arial','FontSize',18,'FontWeight','normal');
saveFigure(f,outDir,'cost_histories');

fid = fopen(fullfile(outDir,'analysis_notes.txt'),'w');
assert(fid~=-1,'Cannot write analysis notes');
fprintf(fid,'CMP02, V0_B_O2_T1, 10 paired seeds, iteration budget 50.\n');
fprintf(fid,'Pre-campaign reference commit: 59da78e2cc56827ecbad87abb7c739c986bd496c\n');
fprintf(fid,'This commit identifies the pre-campaign reference, not a verified execution hash for each run.\n');
fprintf(fid,'Only explicitly listed CMP02 runs are included; seed 42 and connector development runs are excluded.\n');
fprintf(fid,'RRT stops at first solution or budget exhaustion; RRT-star uses the full budget. Compute/time budgets are not equal.\n');
fprintf(fid,'Profiling enabled. Planning wall time includes geometric queries; none is subtracted.\n');
fprintf(fid,'One waypoint per simulation step is a kinematic replay policy, not physical motion timing.\n');
fprintf(fid,'CSV checks cover recorded states; they do not establish continuous collision freedom.\n');
fprintf(fid,'Path lengths are polylines of recorded actual tip positions, not reference positions.\n');
fprintf(fid,'Equal-cost tolerance for pair counts: 1e-9 mm. No global optimality claim.\n');
fprintf(fid,'Among jointly successful pairs, RRT-star shorter: %d; equal within tolerance: %d.\n', ...
    sum(lengthMM(:,2)<lengthMM(:,1)-1e-9),sum(abs(diff(lengthMM,1,2))<=1e-9));
fprintf(fid,'Mean paired length reduction: %.6f percent.\n',mean(gain,'omitnan'));
fprintf(fid,'Reduction of mean lengths (different statistic): %.6f percent.\n', ...
    100*(1-mean(lengthMM(:,2),'omitnan')/mean(lengthMM(:,1),'omitnan')));
fprintf(fid,'Actual execution order is recorded in paired_comparison.csv.\n');
fprintf(fid,'All 20 attempts enter success rates and planning-time summaries. Length summaries condition on success.\n');
fprintf(fid,'Failed paths have NaN lengths, never zero; paired gains require two successful runs.\n');
fprintf(fid,'Successful runs RRT: %d/10; RRT-star: %d/10. Jointly successful pairs: %d.\n',sum(success(:,1)),sum(success(:,2)),sum(all(success,2)));
fprintf(fid,'Geometry mesh identity and source-code identity cannot be established from these CSVs alone.\n');
fclose(fid);
disp(Summary);
fprintf('\nAnalysis complete: 20 runs checked (including planning failures).\nOutputs: %s\n',outDir);

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
