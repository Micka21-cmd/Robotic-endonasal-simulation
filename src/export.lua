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

    return self
end

function Export:record(state)
    assert(state.clearance ~= nil, 'Clearance unavailable for recording')

    local row = {
        self.count,
        state.replayTime,
        state.progressTime,
        state.progressTime / self.config.duration,
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
    }
end

function Export:writeConnector(stem, runId)
    local connector = self.connector

    -- No connector result was recorded, e.g. invalid initial state.
    if connector == nil then
        return
    end

    local config = self.config

    local pathHeader = {
        'sample',
        'q1_rad', 'q2_rad', 'q3_rad',
        'q4_rad', 'q5_rad', 'q6_rad',
    }

    local summary = {
        {'run_id', runId},
        {'run_label', config.runLabel},
        {'target_name', config.targetName},
        {'method', 'local_cartesian_connection_ik'},
        {'status', connector.success and 'success' or 'failed'},
        {'reason', connector.reason},
        {'returned_configurations', #connector.rows},
        {'path_replayed', 0},
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

    writeCsv(
        stem .. '_connector_path.csv',
        pathHeader,
        connector.rows
    )

    writeCsv(
        stem .. '_connector_summary.csv',
        {'parameter', 'value'},
        summary
    )

    self.logger(
        'Connector path export: ' .. stem .. '_connector_path.csv'
    )
    self.logger(
        'Connector summary export: ' .. stem .. '_connector_summary.csv'
    )
end

function Export:write(context)
    local config = self.config
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
        'robot_wall_collision'
    }

    writeCsv(stem .. '_states.csv', header, self.rows)

    local summary = {
        {'run_label', config.runLabel},
        {'status', context.status},
        {'reason', context.reason},
        {'accepted_samples', self.count},
        {'nominal_duration_s', config.duration},
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

    writeCsv(stem .. '_summary.csv', {'parameter', 'value'}, summary)
    self:writeConnector(stem, runId)

    self.logger('State export: ' .. stem .. '_states.csv')
    self.logger('Summary export: ' .. stem .. '_summary.csv')
    self.logger(string.format(
        'Export completed: %d configurations; status=%s',
        self.count,
        context.status
    ))
end

return Export