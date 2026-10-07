-- Accepted-state recording, statistics and CSV export.
-- This module does not access or modify the simulation scene.

local Export = {}
Export.__index = Export

local function csvCell(value)
    if type(value) == 'number' then
        return (string.format('%.15g', value):gsub(',', '.'))
    end

    local text = tostring(value)
    return '"' .. text:gsub('"', '""') .. '"'
end

local function writeCsv(path, header, rows)
    local file, openError = io.open(path, 'w')
    if not file then
        error('Cannot open file: ' .. path .. ': ' .. tostring(openError))
    end

    local ok, writeError = pcall(function()
        local function writeRow(row)
            local cells = {}

            for i, value in ipairs(row) do
                cells[i] = csvCell(value)
            end

            assert(file:write(table.concat(cells, ',') .. '\n'))
        end

        writeRow(header)

        for _, row in ipairs(rows) do
            writeRow(row)
        end
    end)

    local closed, closeError = file:close()

    if not ok then
        error(writeError)
    end

    if not closed then
        error(closeError)
    end
end

function Export.new(config, runDirectory, logger)
    local self = setmetatable({}, Export)

    self.config = config
    self.runDirectory = runDirectory
    self.logger = logger

    self.rows = {}
    self.count = 0
    self.maxRcmError = 0
    self.maxTipError = 0
    self.minClearance = math.huge
    self.tipPathLength = 0
    self.referencePathLength = 0

    self.previousTipPosition = nil
    self.previousTargetPosition = nil
    return self
end

local function distance3(a, b)
    return math.sqrt(
        (b[1] - a[1])^2
        + (b[2] - a[2])^2
        + (b[3] - a[3])^2
    )
end

local function copyPosition(position)
    return {position[1], position[2], position[3]}
end

function Export:record(state)
    assert(state.clearance ~= nil, 'Clearance unavailable for recording')

    local row = {
        self.count,
        state.replayTime,
        state.progressTime,
        state.pathFraction or (state.progressTime / self.config.duration),
        state.ikAttempts
    }

    for i = 1, 6 do
        row[#row + 1] = state.configuration[i]
    end

    for i = 1, 3 do
        row[#row + 1] = state.tipPosition[i]
    end

    for i = 1, 3 do
        row[#row + 1] = state.targetPosition[i]
    end

    row[#row + 1] = state.rcmError
    row[#row + 1] = state.tipError
    row[#row + 1] = state.clearance
    row[#row + 1] = state.collision and 1 or 0

    if self.config.obstacleEnabled then
        local distance = state.obstacleClearance

        assert(
            type(distance) == 'number'
                and distance == distance
                and math.abs(distance) < math.huge,
            'Obstacle clearance unavailable or invalid for recording'
        )

        row[#row + 1] = distance
        row[#row + 1] = state.obstacleCollision and 1 or 0

        self.minObstacleClearance = math.min(
            self.minObstacleClearance or math.huge,
            distance
        )
    else
        row[#row + 1] = ''
        row[#row + 1] = ''
    end

    if self.previousTipPosition ~= nil then
        self.tipPathLength = self.tipPathLength + distance3(
            self.previousTipPosition,
            state.tipPosition
        )

        self.referencePathLength =
            self.referencePathLength + distance3(
                self.previousTargetPosition,
                state.targetPosition
            )
    end

    self.previousTipPosition = copyPosition(state.tipPosition)
    self.previousTargetPosition = copyPosition(state.targetPosition)

    self.rows[#self.rows + 1] = row

    -- Statistics describe the accepted states recorded above.
    self.count = #self.rows
    self.maxRcmError = math.max(self.maxRcmError, state.rcmError)
    self.maxTipError = math.max(self.maxTipError, state.tipError)
    self.minClearance = math.min(self.minClearance, state.clearance)
end

function Export:recordConnector(connection, restoration)
    assert(
        type(connection.success) == 'boolean',
        'Connector result must contain a success flag'
    )

    local rows = {}

    if connection.success then
        assert(
            type(connection.path) == 'table'
                and #connection.path > 0,
            'Successful connector result must contain a path'
        )

        for index, configuration in ipairs(connection.path) do
            assert(#configuration == 6,
                'Connector configuration must contain six joints')

            local row = {index - 1}

            for joint = 1, 6 do
                local value = configuration[joint]

                assert(
                    type(value) == 'number'
                        and value == value
                        and math.abs(value) < math.huge,
                    'Connector joint values must be finite'
                )

                row[#row + 1] = value
            end

            rows[#rows + 1] = row
        end
    end

    -- Copy data so later changes cannot alter the recorded result.
    self.connector = {
        success = connection.success,
        reason = connection.reason or '',
        rows = rows,
        restorationPassed = restoration.passed,
        jointError = restoration.jointError,
        targetPositionError = restoration.targetPositionError,
        targetQuaternionError = restoration.targetQuaternionError,
        method = connection.method or 'local_cartesian_connection_ik',
    }
end

function Export:writeConnector(stem, runId, context)
    local connector = self.connector

    -- No connector result was recorded, e.g. invalid initial state.
    if connector == nil then
        return
    end

    local config = self.config

    local prefix = ({
        rrt_replay = '_rrt',
        rrt_star_replay = '_rrt_star',
    })[config.executionMode] or '_connector'

    local pathFile = stem .. prefix .. '_path.csv'
    local summaryFile = stem .. prefix .. '_summary.csv'

    local pathHeader = {
        'sample',
        'q1_rad', 'q2_rad', 'q3_rad',
        'q4_rad', 'q5_rad', 'q6_rad',
    }

    local summary = {
        {'run_id', runId},
        {'run_label', config.runLabel},
        {'target_name', config.targetName},
        {'method', connector.method},
        {'status', connector.success and 'success' or 'failed'},
        {'reason', connector.reason},
        {'returned_configurations', #connector.rows},
        {'replay_started', context.replayStarted and 1 or 0},
        {'path_replayed', context.pathReplayed and 1 or 0},
        {'replay_status', context.replayStarted and context.status or 'not_started'},
        {'replay_accepted_samples', context.replayStarted and self.count or 0},
        {'position_frame', 'Anatomy_V0'},
        {'local_connector_step_m', config.localConnectorStepM},
        {'local_connector_max_steps', config.localConnectorMaxSteps},
        {'segment_resolution_rad', config.segmentResolutionRad},
        {'segment_min_intervals', 2},
        {'max_joint_step_rad', config.maxJointStep},
        {'max_rcm_error_allowed_m', config.maxRcmError},
        {'max_tip_error_allowed_m', config.maxTipError},
        {'min_clearance_allowed_m', config.minClearance},
        {'ik_damping', config.ikDamping},
        {'ik_max_iterations', config.ikMaxIterations},
        {'ik_position_tolerance_m', config.ikPositionTolerance},
        {'ik_orientation_tolerance_rad', config.ikOrientationTolerance},
        {'restoration_passed', connector.restorationPassed and 1 or 0},
        {'restoration_joint_error_rad', connector.jointError},
        {'restoration_target_position_error_m',
            connector.targetPositionError},
        {'restoration_target_quaternion_difference',
            connector.targetQuaternionError},
    }

    writeCsv(pathFile, pathHeader, connector.rows)

    writeCsv(
        summaryFile,
        {'parameter', 'value'},
        summary
    )

    self.logger('Calculated path export: ' .. pathFile)
    self.logger('Path summary export: ' .. summaryFile)
end

function Export:write(context)
    local config = self.config
    local plannerScope = config.executionMode == 'rrt_star_replay'
        and 'RRTStar.plan' or 'RRT.plan'
    local startedAt = context.startedAt or os.time()

    local runDir, runId = self.runDirectory.create(
        config.exportDir,
        config.runLabel,
        startedAt
    )

    local stem = runDir .. '/' .. config.runLabel

    local header = {
        'sample', 'replay_time_s', 'progress_time_s', 'tau',
        'ik_attempts',
        'q1_rad', 'q2_rad', 'q3_rad',
        'q4_rad', 'q5_rad', 'q6_rad',
        'tip_x_m', 'tip_y_m', 'tip_z_m',
        'target_x_m', 'target_y_m', 'target_z_m',
        'rcm_error_m', 'tip_error_m', 'clearance_m',
        'robot_wall_collision',
        'shaft_obstacle_clearance_m',
        'robot_obstacle_collision',
    }

    writeCsv(stem .. '_states.csv', header, self.rows)

    local summary = {
        {'run_label', config.runLabel},
        {'status', context.status},
        {'reason', context.reason},
        {'accepted_samples', self.count},
        {'execution_mode', context.executionMode},
         {'nominal_duration_s',
            context.executionMode ~= 'prescribed_motion'
                and 'not_applicable' or config.duration},

        {'replay_policy',
            context.executionMode ~= 'prescribed_motion'
                and 'one_waypoint_per_simulation_step'
                or 'prescribed_cartesian_motion'},

        {'tau_definition',
            context.executionMode ~= 'prescribed_motion'
                and 'accepted_path_index_fraction'
                or 'progress_time_over_nominal_duration'},
        {'segment_resolution_rad', config.segmentResolutionRad},
        {'segment_min_intervals', 2},
        {'replay_time_s', context.replayTime},
        {'progress_time_s', context.progressTime},
        {'max_rcm_error_allowed_m', config.maxRcmError},
        {'max_tip_error_allowed_m', config.maxTipError},
        {'min_clearance_allowed_m', config.minClearance},
        {'max_joint_step_rad', config.maxJointStep},
        {'position_frame', 'Anatomy_V0'},
        {'dynamics_enabled_during_tracking', 0},
        {'run_id', runId},
        {'started_at_local', os.date('%Y-%m-%dT%H:%M:%S', startedAt)},
        {'method', config.method},
        {'run_note', config.runNote},
        {'ik_damping', config.ikDamping},
        {'ik_max_iterations', config.ikMaxIterations},
        {'ik_position_tolerance_m', config.ikPositionTolerance},
        {'ik_orientation_tolerance_rad', config.ikOrientationTolerance},
        {'max_ik_attempts', config.maxIkAttempts},
        {'target_name', config.targetName},
        {'target_path', config.targetPath}
    }

    if context.startPosition and context.goalPosition then
        local axes = {'x', 'y', 'z'}

        for i = 1, 3 do
            summary[#summary + 1] = {
                'start_' .. axes[i] .. '_m',
                context.startPosition[i]
            }

            summary[#summary + 1] = {
                'goal_' .. axes[i] .. '_m',
                context.goalPosition[i]
            }
        end
    end

    -- Avoid reporting extrema when no state was recorded.
    if self.count > 0 then
        summary[#summary + 1] = {
            'max_rcm_error_m', self.maxRcmError
        }
        summary[#summary + 1] = {
            'max_tip_error_m', self.maxTipError
        }
        summary[#summary + 1] = {
            'min_clearance_m', self.minClearance
        }
    end

    if context.initialAnatomyPose then
        local names = {'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw'}

        for i = 1, 7 do
            summary[#summary + 1] = {
                'anatomy_world_' .. names[i],
                context.initialAnatomyPose[i]
            }
        end
    end

    if context.finalTipError ~= nil then
        summary[#summary + 1] = {
            'final_tip_to_goal_error_m', context.finalTipError
        }
    end

    if context.plannerStats then
        local stats = context.plannerStats

        local statsPrefix = context.executionMode == 'rrt_star_replay'
            and 'rrt_star_' or 'rrt_'

        for _, name in ipairs({
            'seed',
            'iterations',
            'nodes',
            'connector_calls',
            'rejected_connections',
        }) do
            summary[#summary + 1] = {
                statsPrefix .. name, stats[name]
            }
        end

        summary[#summary + 1] = {
            'rrt_max_iterations', config.rrtMaxIterations
        }
        summary[#summary + 1] = {
            'rrt_extension_m', config.rrtExtensionM
        }
        summary[#summary + 1] = {
            'rrt_goal_bias', config.rrtGoalBias
        }
        summary[#summary + 1] = {
            'rrt_sampling_frame', 'Anatomy_V0'
        }
        summary[#summary + 1] = {
            'rrt_nearest_metric', 'euclidean_tip_position'
        }
        summary[#summary + 1] = {
            'rrt_random_generator', 'park_miller_16807'
        }

        for i, axis in ipairs({'x', 'y', 'z'}) do
            summary[#summary + 1] = {
                'rrt_bounds_min_' .. axis .. '_m',
                config.rrtBoundsMin[i]
            }
            summary[#summary + 1] = {
                'rrt_bounds_max_' .. axis .. '_m',
                config.rrtBoundsMax[i]
            }
        end
    end

    summary[#summary + 1] = {
        'obstacle_enabled', config.obstacleEnabled and 1 or 0
    }

    if config.obstacleEnabled then
        summary[#summary + 1] = {
            'obstacle_path', config.obstaclePath
        }

        summary[#summary + 1] = {
            'min_obstacle_clearance_allowed_m',
            config.minObstacleClearance
        }

        if self.minObstacleClearance ~= nil then
            summary[#summary + 1] = {
                'min_shaft_obstacle_clearance_m',
                self.minObstacleClearance
            }
        end

        if context.obstaclePose then
            local names = {
                'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw'
            }

            for i, name in ipairs(names) do
                summary[#summary + 1] = {
                    'obstacle_anatomy_' .. name,
                    context.obstaclePose[i]
                }
            end
        end

        if context.obstacleDimensions then
            for i, axis in ipairs({'x', 'y', 'z'}) do
                summary[#summary + 1] = {
                    'obstacle_size_' .. axis .. '_m',
                    context.obstacleDimensions[i]
                }
            end
        end
    end

    if context.planningWallTime ~= nil then
        summary[#summary + 1] = {
            'planning_wall_time_s',
            context.planningWallTime
        }

        summary[#summary + 1] = {
            'planning_time_scope',
            plannerScope .. ' including connections and path reconstruction'
        }
    end

    if self.count > 0 then
        summary[#summary + 1] = {
            'recorded_tip_path_length_m',
            self.tipPathLength
        }

        summary[#summary + 1] = {
            'recorded_reference_path_length_m',
            self.referencePathLength
        }

        summary[#summary + 1] = {
            'path_length_definition',
            'sum of Euclidean distances between consecutive recorded positions'
        }

        summary[#summary + 1] = {
            'recorded_path_complete',
            context.status == 'completed' and 1 or 0
        }
    end

    summary[#summary + 1] = {
        'planning_profile_enabled',
        config.profilePlanning and 1 or 0
    }

    if context.planningProfile then
        summary[#summary + 1] = {
            'planning_profile_scope',
            plannerScope .. ' only; replay and CSV export excluded'
        }

        for _, row in ipairs(context.planningProfile) do
            local prefix = 'profile_' .. row.name

            summary[#summary + 1] = {
                prefix .. '_calls', row.calls
            }

            summary[#summary + 1] = {
                prefix .. '_inclusive_s', row.inclusive
            }

            summary[#summary + 1] = {
                prefix .. '_exclusive_s', row.exclusive
            }

            summary[#summary + 1] = {
                prefix .. '_lua_errors', row.errors
            }
        end
    end

    if context.executionMode == 'rrt_star_replay' and context.plannerStats then
        local stats = context.plannerStats
        for _, name in ipairs({
            'fixed_connector_calls', 'parent_changes', 'rewires',
            'descendant_cost_updates', 'goal_connections', 'goal_candidates',
            'solution_improvements', 'out_of_bounds_extensions',
            'first_solution_iteration', 'first_solution_connector_calls',
            'first_solution_cost_m', 'best_cost_m', 'last_radius_m',
            'cost_definition', 'stop_reason',
        }) do
            local value = stats[name]
            if value == nil then value = 'not_applicable' end
            summary[#summary + 1] = {'rrt_star_' .. name, value}
        end
        summary[#summary + 1] = {
            'rrt_star_radius_max_m', config.rrtStarRadiusMaxM
        }
        summary[#summary + 1] = {
            'rrt_star_gamma_m', config.rrtStarGammaM
        }
        summary[#summary + 1] = {
            'rrt_star_neighbour_dimension', 3
        }
        summary[#summary + 1] = {
            'rrt_star_cost_history_policy', 'first_solution_and_strict_improvements'
        }
        local historyFile = stem .. '_rrt_star_cost_history.csv'
        writeCsv(historyFile,
            {'iteration', 'connector_calls', 'best_cost_m'},
            context.plannerCostHistory or {}
        )
        self.logger('RRT* cost history export: ' .. historyFile)
    end

    writeCsv(stem .. '_summary.csv', {'parameter', 'value'}, summary)
    self:writeConnector(stem, runId, context)

    self.logger('State export: ' .. stem .. '_states.csv')
    self.logger('Summary export: ' .. stem .. '_summary.csv')
    self.logger(string.format(
        'Export completed: %d configurations; status=%s',
        self.count,
        context.status
    ))
end

return Export