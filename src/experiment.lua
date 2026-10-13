-- Coordinate a prescribed-motion experiment.
-- Planning, IK, validation and recording remain separate responsibilities.

local Experiment = {}
Experiment.__index = Experiment

local function norm(vector)
    return math.sqrt(
        vector[1]^2 + vector[2]^2 + vector[3]^2
    )
end

function Experiment.new(simApi, ikApi, config, modules)
    local self = setmetatable({}, Experiment)

    self.sim = simApi
    self.ik = ikApi
    self.config = config
    self.modules = modules

    self.running = false
    self.elapsed = 0
    self.replayTime = 0
    self.status = 'incomplete'
    self.reason = 'Stopped before completion, or script error'
    self.slowdownReported = false

    return self
end

function Experiment:message(text)
    self.sim.addLog(self.sim.verbosity_scriptinfos, text)
end

function Experiment:abort(reason)
    self.running = false
    self.status = 'failed'
    self.reason = reason

    self.sim.addLog(
        self.sim.verbosity_errors,
        'TEST INTERRUPTED: ' .. reason
    )

    self.sim.pauseSimulation()
end

function Experiment:profilePlanning(work)
    local sim = self.sim
    local originalValidatorSim = self.validator.sim

    -- Separate labels for the four geometric queries.
    local queries = {
        collision_wall = originalValidatorSim.checkCollision,
        distance_wall = originalValidatorSim.checkDistance,
        collision_obstacle = originalValidatorSim.checkCollision,
        distance_obstacle = originalValidatorSim.checkDistance,
    }

    -- Only the validator receives this proxy.
    -- The global sim API is not modified.
    local proxy = setmetatable({}, {
        __index = originalValidatorSim
    })

    proxy.checkCollision = function(a, b, ...)
        if b == self.wall then
            return queries.collision_wall(a, b, ...)
        elseif b == self.obstacle then
            return queries.collision_obstacle(a, b, ...)
        end

        return originalValidatorSim.checkCollision(a, b, ...)
    end

    proxy.checkDistance = function(a, b, ...)
        if b == self.wall then
            return queries.distance_wall(a, b, ...)
        elseif b == self.obstacle then
            return queries.distance_obstacle(a, b, ...)
        end

        return originalValidatorSim.checkDistance(a, b, ...)
    end

    local targets = {}

    local function add(owner, key, name)
        targets[#targets + 1] = {
            owner = owner,
            key = key,
            name = name,
        }
    end

    add(self.localConnector, 'connect', 'connector_total')
    add(self.localConnector, 'connectConfiguration', 'connector_fixed_total')
    add(self.ikSolver, 'solveCandidate', 'ik_solve')
    add(self.ikSolver, 'syncFromScene', 'ik_sync')
    add(self.robot, 'setConfiguration', 'robot_set')
    add(self.modules.SegmentValidation, 'check', 'segment_check')
    add(self.validator, 'checkJoints', 'joint_check')
    add(self.validator, 'checkState', 'state_check')
    add(self.validator, 'measureState', 'measure_wall_state')
    add(self.validator, 'measureObstacle', 'measure_obstacle')

    for _, name in ipairs({
        'collision_wall',
        'distance_wall',
        'collision_obstacle',
        'distance_obstacle',
    }) do
        add(queries, name, name)
    end

    self.validator.sim = proxy

    local ok, rows, result = pcall(function()
        return self.modules.Profiler.run(
            function()
                return sim.getSystemTime()
            end,
            targets,
            work
        )
    end)

    -- Restore the validator API even if profiling or planning fails.
    self.validator.sim = originalValidatorSim

    if not ok then
        error(rows, 0)
    end

    self.planningProfile = rows
    return result
end

-- Static check of one previously recorded T1 configuration, not a path test.
function Experiment:diagnoseKnownT1()
    local sim, config = self.sim, self.config
    assert(config.targetName == 'T1', 'T1 diagnostic requires targetName=T1')
    assert(config.obstacleEnabled and self.obstacle ~= nil,
        'T1 diagnostic requires the current obstacle enabled')
    assert(norm({self.goalPosition[1] - 0.003,
        self.goalPosition[2], self.goalPosition[3] - 0.075}) < 1e-6,
        'Current T1 differs from the reference anatomical position')

    -- Last row (sample 320) of the supplied V0_B_T1_states(4).csv.
    local goalQ = {
        -0.00332872669655434, -0.566213262474279,
        1.13807181493872, -1.03745910130746,
        0.789170202869579, -1.33906264703047e-15
    }
    local referenceTip = {
        0.00300000166324943, -1.71724534592244e-09,
        0.0749999970457265
    }
    local savedQ = self.robot:readConfiguration()
    local savedTarget = sim.getObjectPose(self.target, sim.handle_world)
    local function copy(t)
        local r = {}; for i,v in ipairs(t) do r[i] = v end; return r
    end
    savedQ, savedTarget = copy(savedQ), copy(savedTarget)

    local ok, result = pcall(function()
        -- Identical arguments check joint limits only, not a move from T0.
        local jointsValid, jointReason = self.validator:checkJoints(goalQ, goalQ)
        if not jointsValid then
            return {jointsValid = false, reason = jointReason}
        end
        self.robot:setConfiguration(goalQ)
        local referencePose = self.modules.Trajectory.poseAt(
            1, 1, self.goalPosition, self.goalPosition
        )
        sim.setObjectPose(self.target, referencePose, self.anatomy)
        local tipPosition = sim.getObjectPosition(self.tip, self.anatomy)
        local rcm, tipError, wallClearance, onShaft, wallCollision =
            self.validator:measureState(0)
        local obstacleClearance, obstacleCollision =
            self.validator:measureObstacle(0)
        local valid, reason = self.validator:checkState()
        local referenceError = norm({
            tipPosition[1] - referenceTip[1],
            tipPosition[2] - referenceTip[2],
            tipPosition[3] - referenceTip[3]
        })
        return {
            jointsValid = true, valid = valid, reason = reason,
            tipPosition = tipPosition, referenceError = referenceError,
            rcm = rcm, tipError = tipError,
            onShaft = onShaft, wallClearance = wallClearance,
            wallCollision = wallCollision, obstacleClearance = obstacleClearance,
            obstacleCollision = obstacleCollision
        }
    end)

    -- Attempt every restoration even if one restoration operation throws.
    local restorationErrors = {}
    local function restore(label, action)
        local success, err = pcall(action)
        if not success then
            restorationErrors[#restorationErrors + 1] = label .. ': ' .. tostring(err)
        end
    end
    restore('robot', function() self.robot:setConfiguration(savedQ) end)
    restore('target', function()
        sim.setObjectPose(self.target, savedTarget, sim.handle_world)
    end)
    restore('IK', function() self.ikSolver:syncFromScene() end)
    assert(#restorationErrors == 0,
        'T1 diagnostic restoration error: ' .. table.concat(restorationErrors, '; '))

    local afterQ = self.robot:readConfiguration()
    local afterTarget = sim.getObjectPose(self.target, sim.handle_world)
    local jointError, positionError, quaternionMinus, quaternionPlus = 0, 0, 0, 0
    for i = 1, #savedQ do
        jointError = math.max(jointError, math.abs(afterQ[i] - savedQ[i]))
    end
    for i = 1, 3 do positionError = positionError + (afterTarget[i] - savedTarget[i])^2 end
    for i = 4, 7 do
        quaternionMinus = quaternionMinus + (afterTarget[i] - savedTarget[i])^2
        quaternionPlus = quaternionPlus + (afterTarget[i] + savedTarget[i])^2
    end
    positionError = math.sqrt(positionError)
    local quaternionError = math.sqrt(math.min(quaternionMinus, quaternionPlus))
    local restored = jointError <= 1e-9 and positionError <= 1e-9 and quaternionError <= 1e-9
    self:message(string.format(
        'T1 diagnostic restoration: %s; joint=%.3e rad; target=%.3e m; quaternion=%.3e.',
        restored and 'PASS' or 'FAIL', jointError, positionError, quaternionError
    ))
    assert(restored, 'T1 diagnostic restoration mismatch')
    if not ok then error(result, 0) end

    if not result.jointsValid then
        self:message('T1 STATIC DIAGNOSTIC: REJECTED; ' .. tostring(result.reason))
        return result
    end
    local function mm(value)
        if type(value) ~= 'number' or value ~= value or math.abs(value) == math.huge then
            return 'unavailable'
        end
        return string.format('%.9f mm', 1000 * value)
    end
    self:message(string.format(
        'T1 static tip position (Anatomy_V0): [%.9f, %.9f, %.9f] mm.',
        1000 * result.tipPosition[1], 1000 * result.tipPosition[2], 1000 * result.tipPosition[3]
    ))
    self:message('T1 reference-position difference: ' .. mm(result.referenceError))
    self:message('T1 RCM error: ' .. mm(result.rcm)
        .. '; tip-to-goal: ' .. mm(result.tipError)
        .. '; RCM on physical shaft: ' .. tostring(result.onShaft))
    self:message('T1 shaft-wall clearance: ' .. mm(result.wallClearance)
        .. '; minimum: ' .. mm(config.minClearance)
        .. '; robot-wall collision: ' .. tostring(result.wallCollision))
    self:message('T1 shaft-obstacle clearance: ' .. mm(result.obstacleClearance)
        .. '; minimum: ' .. mm(config.minObstacleClearance)
        .. '; robot-obstacle collision: ' .. tostring(result.obstacleCollision))
    local referenceMatches = result.referenceError <= 1e-6
    local verdict = not referenceMatches and 'REFERENCE_MISMATCH'
        or (result.valid and 'ACCEPTED' or 'REJECTED')
    self:message('T1 STATIC DIAGNOSTIC: ' .. verdict .. '; '
        .. (result.reason or 'Current state checks passed.')
        .. ' One stored configuration only; no path was tested.')
    return result
end

-- Static endpoint preflight: one IK candidate per goal, no path is tested.
function Experiment:diagnoseEndpoints()
    local sim, config = self.sim, self.config
    local function copy(t)
        local r = {}; for i, v in ipairs(t) do r[i] = v end; return r
    end
    local savedQ = copy(self.robot:readConfiguration())
    local savedTarget = copy(sim.getObjectPose(self.target, sim.handle_world))
    local function restore()
        local errors = {}
        local function attempt(label, action)
            local ok, err = pcall(action)
            if not ok then errors[#errors + 1] = label .. ': ' .. tostring(err) end
        end
        attempt('robot', function() self.robot:setConfiguration(savedQ) end)
        attempt('target', function()
            sim.setObjectPose(self.target, savedTarget, sim.handle_world)
        end)
        attempt('IK', function() self.ikSolver:syncFromScene() end)
        assert(#errors == 0, table.concat(errors, '; '))
        local q = self.robot:readConfiguration()
        local pose = sim.getObjectPose(self.target, sim.handle_world)
        local dq, dp, qm, qp = 0, 0, 0, 0
        for i = 1, #savedQ do dq = math.max(dq, math.abs(q[i] - savedQ[i])) end
        for i = 1, 3 do dp = dp + (pose[i] - savedTarget[i])^2 end
        for i = 4, 7 do
            qm = qm + (pose[i] - savedTarget[i])^2
            qp = qp + (pose[i] + savedTarget[i])^2
        end
        dp = math.sqrt(dp)
        local quaternionError = math.sqrt(math.min(qm, qp))
        assert(dq <= 1e-9 and dp <= 1e-9 and quaternionError <= 1e-9,
            'Endpoint preflight restoration mismatch')
        self:message(string.format(
            'Endpoint restoration: PASS; joint=%.3e rad; target=%.3e m; quaternion=%.3e.',
            dq, dp, quaternionError))
    end
    local function mm(value)
        if type(value) ~= 'number' or value ~= value or math.abs(value) == math.huge then
            return 'unavailable'
        end
        return string.format('%.9f mm', 1000 * value)
    end
    local accepted = 0
    for _, name in ipairs({'T0', 'T1', 'T2'}) do
        -- Each goal starts from the same scene configuration, independent of the previous test.
        local ok, result = pcall(function()
            local p = name == 'T0' and self.startPosition
                or sim.getObjectPosition(sim.getObject('/Anatomy_V0/target_' .. name), self.anatomy)
            assert(p[3] > 0, 'Goal anatomical Z must be positive')
            self:message(string.format('%s commanded position: [%.6f, %.6f, %.6f] mm.',
                name, 1000*p[1], 1000*p[2], 1000*p[3]))
            sim.setObjectPose(self.target,
                self.modules.Trajectory.poseAt(1, 1, p, p), self.anatomy)
            local candidate = savedQ
            if name ~= 'T0' then
                local flags
                candidate, flags = self.ikSolver:solveCandidate()
                if candidate == nil then
                    return {valid = false, reason = 'IK failed; flags=' .. tostring(flags)}
                end
            end
            -- Identical arguments check limits only; this is not a motion from T0.
            local jointsValid, jointReason = self.validator:checkJoints(candidate, candidate)
            if not jointsValid then return {valid = false, reason = jointReason} end
            self.robot:setConfiguration(candidate)
            local rcm, tipError, wall, onShaft, wallCollision = self.validator:measureState(0)
            local obstacle, obstacleCollision = self.validator:measureObstacle(0)
            local valid, reason = self.validator:checkState()
            self:message(name .. ' RCM=' .. mm(rcm) .. '; tip-to-goal=' .. mm(tipError)
                .. '; RCM on physical shaft=' .. tostring(onShaft))
            self:message(name .. ' shaft-wall=' .. mm(wall)
                .. '; minimum=' .. mm(config.minClearance)
                .. '; robot-wall collision=' .. tostring(wallCollision))
            self:message(name .. ' shaft-obstacle=' .. mm(obstacle)
                .. '; minimum=' .. mm(config.minObstacleClearance)
                .. '; robot-obstacle collision=' .. tostring(obstacleCollision))
            local qText = {}
            for i, v in ipairs(candidate) do qText[i] = string.format('%.17g', v) end
            self:message(name .. ' static configuration (rad): {' .. table.concat(qText, ', ') .. '}')
            return {valid = valid, reason = reason}
        end)
        restore() -- Also required on an IK/measurement exception.
        if not ok then error(result, 0) end
        if result.valid then accepted = accepted + 1 end
        self:message(name .. ' STATIC PREFLIGHT: ' .. (result.valid and 'ACCEPTED' or 'REJECTED')
            .. '; ' .. (result.reason or 'State checks passed.'))
    end
    self:message(string.format(
        'ENDPOINT PREFLIGHT: %d/3 accepted. One configuration per position; no path tested.', accepted))
    return accepted == 3
end

function Experiment:initialize()
    local sim = self.sim
    local config = self.config

    assert(
        config.executionMode == 'prescribed_motion'
            or config.executionMode == 'connector_replay'
            or config.executionMode == 'rrt_replay'
            or config.executionMode == 'rrt_star_replay',
        'Unsupported execution mode'
    )

    self.startedAt = os.time()

    self.recorder = self.modules.Export.new(
        config,
        self.modules.RunDirectory,
        function(text)
            self:message(text)
        end
    )

    local root = sim.getObject('..')

    self.anatomy = sim.getObject('/Anatomy_V0')
    self.shaft = sim.getObject('/UR3/Instrument_shaft')
    self.tip = sim.getObject('/UR3/tool_tip')
    self.target = sim.getObject('/tool_target')
    self.wall = sim.getObject('/Anatomy_V0/Corridor_wall')

    if config.obstacleEnabled then
        self.obstacle = sim.getObject(config.obstaclePath)

        self.initialObstaclePose = sim.getObjectPose(
            self.obstacle, self.anatomy
        )

        local _, _, dimensions = sim.getShapeGeomInfo(self.obstacle)
        self.obstacleDimensions = {
            dimensions[1],
            dimensions[2],
            dimensions[3],
        }
    end

    local goal = sim.getObject(config.targetPath)

    self.startPosition = {
        config.startPosition[1],
        config.startPosition[2],
        config.startPosition[3]
    }

    self.goalPosition = sim.getObjectPosition(goal, self.anatomy)

    assert(
        self.startPosition[3] > 0 and self.goalPosition[3] > 0,
        'Start and goal must have positive anatomical Z'
    )

    self.initialAnatomyPose = sim.getObjectPose(
        self.anatomy,
        sim.handle_world
    )

    self.robot = self.modules.Robot.new(sim, root, '../joint', 6)

    self.savedDynamics = sim.getBoolParam(
        sim.boolparam_dynamics_handling_enabled
    )

    self.savedTargetPose = sim.getObjectPose(
        self.target,
        sim.handle_world
    )

    sim.setBoolParam(sim.boolparam_dynamics_handling_enabled, false)

    local p = sim.getObjectPosition(self.tip, self.anatomy)
    local startError = norm({
        p[1] - self.startPosition[1],
        p[2] - self.startPosition[2],
        p[3] - self.startPosition[3]
    })

    if startError > 0.0001 then
        self:abort('Tool tip is not at T0 at initialization.')
        return
    end

    sim.setObjectPose(
        self.target,
        sim.getObjectPose(self.tip, sim.handle_world),
        sim.handle_world
    )

    self.robotCollection = sim.createCollection(1)
    sim.addItemToCollection(
        self.robotCollection,
        sim.handle_tree,
        root,
        0
    )

    self.validator = self.modules.Validation.new(
        sim,
        self.robot,
        {
            anatomy = self.anatomy,
            shaft = self.shaft,
            tip = self.tip,
            target = self.target,
            wall = self.wall,
            obstacle = self.obstacle,
            robotCollection = self.robotCollection
        },
        config
    )

    self.ikSolver = self.modules.Kinematics.new(
        self.ik,
        self.robot,
        config
    )

    self.ikSolver:initialize(self.tip, self.target)

    -- Run before the normal initial-state gate so all three endpoints are reported.
    if config.diagnosticEndpoints == true then
        self.diagnosticOnly = true
        self.running = false
        local ok, result = pcall(function() return self:diagnoseEndpoints() end)
        if not ok then
            sim.addLog(sim.verbosity_errors, 'ENDPOINT PREFLIGHT ERROR: ' .. tostring(result))
        end
        sim.stopSimulation()
        return
    end

    local valid, reason = self.validator:checkState()
    if not valid then
        self:abort(reason)
        return
    end

    if config.obstacleEnabled then
        local clearance = self.validator:measureObstacle()

        self:message(string.format(
            'Initial shaft-obstacle clearance: %.4f mm; minimum: %.4f mm.',
            1000 * clearance,
            1000 * config.minObstacleClearance
        ))
    end

    -- Temporary static diagnostic; exits before any planning or replay.
    if config.diagnosticGoalT1 == true then
        self.diagnosticOnly = true
        self.running = false
        local ok, err = pcall(function() self:diagnoseKnownT1() end)
        if not ok then
            sim.addLog(sim.verbosity_errors, 'T1 DIAGNOSTIC ERROR: ' .. tostring(err))
        end
        sim.stopSimulation()
        return
    end

    self.localConnector = self.modules.LocalConnector.new(
        sim,
        self.robot,
        self.ikSolver,
        self.validator,
        self.modules,
        {
            anatomy = self.anatomy,
            tip = self.tip,
            target = self.target,
        },
        config
    )

    -- Temporary diagnostic: compute a connection without replaying it.
    local beforeQ = self.robot:readConfiguration()
    local beforeTarget = sim.getObjectPose(
        self.target, sim.handle_world
    )

    local connection

    if config.executionMode == 'rrt_replay'
        or config.executionMode == 'rrt_star_replay' then
        local isStar = config.executionMode == 'rrt_star_replay'
        local plannerName = isStar and 'RRT*' or 'RRT'
        local planner = isStar and self.modules.RRTStar or self.modules.RRT
        assert(planner, 'Planner module was not loaded: ' .. plannerName)

        self:message(string.format(
            '%s planning: seed=%d; iteration budget=%d.',
            plannerName, config.rrtSeed, config.rrtMaxIterations
        ))
        local startPose = sim.getObjectPose(self.tip, self.anatomy)
        local function plan()
            return planner.plan(
                beforeQ, startPose, self.goalPosition,
                self.localConnector, config,
                {log = function(text) self:message(text) end}
            )
        end
        local planningStartedAt = sim.getSystemTime()
        if config.profilePlanning then
            connection = self:profilePlanning(plan)
        else
            connection = plan()
        end
        self.planningWallTime = sim.getSystemTime() - planningStartedAt
        self.plannerStats = connection.stats
        self.plannerCostHistory = connection.costHistory

        self:message(string.format(
            '%s planning wall time: %.6f s; profiling=%s.',
            plannerName, self.planningWallTime,
            config.profilePlanning and 'on' or 'off'
        ))
        if self.planningProfile then
            for _, row in ipairs(self.planningProfile) do
                self:message(string.format(
                    'PROFILE %s: calls=%d; inclusive=%.6f s; '
                        .. 'exclusive=%.6f s; Lua errors=%d.',
                    row.name, row.calls, row.inclusive, row.exclusive, row.errors
                ))
            end
        end
        local stats = connection.stats
        self:message(string.format(
            '%s result: %s; iterations=%d; nodes=%d; '
                .. 'connector calls=%d; rejected connections=%d.',
            plannerName, connection.success and 'SUCCESS' or 'FAILED',
            stats.iterations, stats.nodes, stats.connector_calls,
            stats.rejected_connections
        ))
        if isStar then
            self:message(string.format(
                'RRT* optimization: parent changes=%d; rewires=%d; '
                    .. 'descendant updates=%d; improvements=%d; best=%s mm.',
                stats.parent_changes, stats.rewires, stats.descendant_cost_updates,
                stats.solution_improvements,
                stats.best_cost_m and string.format('%.6f', 1000 * stats.best_cost_m)
                    or 'none'
            ))
        end
    else
        connection = self.localConnector:connect(beforeQ, self.goalPosition)
    end


    -- Temporary prerequisite test for RRT* node connections.
    -- Outside the RRT planning timer and planner statistics.
    if connection.success and #connection.path >= 2 then
        local startQ = connection.path[1]
        local goalQ = connection.path[2]

        local probe = self.localConnector:connectConfiguration(
            startQ,
            goalQ
        )

        assert(
            probe.success,
            'Fixed-node connector test failed: '
                .. tostring(probe.reason)
        )

        local reachedQ = probe.path[#probe.path]
        local endpointError = 0

        for i = 1, #goalQ do
            endpointError = math.max(
                endpointError,
                math.abs(reachedQ[i] - goalQ[i])
            )
        end

        assert(endpointError == 0,
            'Fixed-node connector did not preserve the destination')

        self:message(string.format(
            'Fixed-node connector: PASS; endpoint error=%.3e rad; '
                .. 'correction=%.6f deg; configurations=%d.',
            endpointError,
            math.deg(probe.endpointCorrectionRad),
            #probe.path
        ))
    end

    -- Measure restoration before any further scene changes.
    local afterQ = self.robot:readConfiguration()
    local afterTarget = sim.getObjectPose(
        self.target, sim.handle_world
    )

    local jointError = 0
    for i = 1, #beforeQ do
        jointError = math.max(
            jointError,
            math.abs(afterQ[i] - beforeQ[i])
        )
    end

    local targetPositionError = norm({
        afterTarget[1] - beforeTarget[1],
        afterTarget[2] - beforeTarget[2],
        afterTarget[3] - beforeTarget[3],
    })

    -- Quaternions q and -q represent the same orientation.
    local quaternionDifference = 0
    local quaternionSum = 0

    for i = 4, 7 do
        quaternionDifference = quaternionDifference
            + (afterTarget[i] - beforeTarget[i])^2
        quaternionSum = quaternionSum
            + (afterTarget[i] + beforeTarget[i])^2
    end

    local targetQuaternionError = math.sqrt(
        math.min(quaternionDifference, quaternionSum)
    )

    local restored = jointError <= 1e-9
        and targetPositionError <= 1e-9
        and targetQuaternionError <= 1e-9


    self.recorder:recordConnector(connection, {
        passed = restored,
        jointError = jointError,
        targetPositionError = targetPositionError,
        targetQuaternionError = targetQuaternionError,
    })

    self:message(string.format(
        'Connector restoration: %s; joint=%.3e rad; '
            .. 'target position=%.3e m; quaternion difference=%.3e',
        restored and 'PASS' or 'FAIL',
        jointError,
        targetPositionError,
        targetQuaternionError
    ))

    if not restored then
        self:abort('Local connector did not restore the scene.')
        return
    end

    if not connection.success then
        self:abort(
            'Path calculation failed: '
                .. tostring(connection.reason)
        )
        return
    end

    self:message(string.format(
        'Path calculation T0 -> %s: PASS; %d configurations returned.',
        config.targetName,
        #connection.path
    ))

    if config.executionMode ~= 'prescribed_motion' then
        assert(
            connection.targetPoses
                and #connection.targetPoses == #connection.path,
            'Connector references do not match the path'
        )

        self.connection = connection
        self.pathIndex = 1
        self.pathFraction = 0

        sim.setObjectPose(
            self.target,
            connection.targetPoses[1],
            self.anatomy
        )

        local startValid, startReason = self.validator:checkState()
        if not startValid then
            self:abort('Replay start rejected: ' .. tostring(startReason))
            return
        end

        self:recordAcceptedState(0)
        self.replayStarted = true
        self.running = true

        self:message(string.format(
            'Valid start. Stored path replay: T0 -> %s; '
                .. '%d stored configurations; one waypoint per simulation step.',
            config.targetName,
            #connection.path
        ))
    else
        self:recordAcceptedState(0)
        self.running = true

        self:message(string.format(
            'Valid start. Kinematic RCM test: T0 -> %s; '
                .. 'nominal duration %.1f s.',
            config.targetName,
            config.duration
        ))
    end
end

function Experiment:recordAcceptedState(attempts)
    local q = self.robot:readConfiguration()
    local p = self.sim.getObjectPosition(self.tip, self.anatomy)
    local pd = self.sim.getObjectPosition(self.target, self.anatomy)
    local obstacleClearance, obstacleCollision =
        self.validator:measureObstacle()
    local rcm, tipError, clearance, _, collision =
        self.validator:measureState()

    self.recorder:record({
        replayTime = self.replayTime,
        progressTime = self.elapsed,
        ikAttempts = attempts,
        configuration = q,
        tipPosition = p,
        targetPosition = pd,
        rcmError = rcm,
        tipError = tipError,
        clearance = clearance,
        collision = collision,
        pathFraction = self.pathFraction,
        obstacleClearance = obstacleClearance,
        obstacleCollision = obstacleCollision,
    })
end

function Experiment:step()
    if not self.running then
        return
    end

    if self.config.executionMode ~= 'prescribed_motion' then
        self:stepConnectorReplay()
        return
    end

    local sim = self.sim
    local config = self.config

    local previousQ = self.robot:readConfiguration()
    local previousTarget = sim.getObjectPose(
        self.target,
        sim.handle_world
    )

    local advance = sim.getSimulationTimeStep()
    local nextTime
    local candidate
    local accepted = false
    local lastReason = ''
    local attemptsUsed = 0

    -- Retry IK and joint checks without moving the scene robot.
    for attempt = 1, config.maxIkAttempts do
        attemptsUsed = attempt
        nextTime = math.min(config.duration, self.elapsed + advance)

        local targetPose = self.modules.Trajectory.poseAt(
            nextTime,
            config.duration,
            self.startPosition,
            self.goalPosition
        )

        sim.setObjectPose(self.target, targetPose, self.anatomy)

        local flags
        candidate, flags = self.ikSolver:solveCandidate()

        if candidate ~= nil then
            local acceptable, reason =
                self.validator:checkJoints(candidate, previousQ)

            if acceptable then
                accepted = true
                break
            end

            lastReason = reason
        else
            lastReason = 'IK failed, flags=' .. tostring(flags)
        end

        advance = advance / 2
    end

    if not accepted then
        sim.setObjectPose(
            self.target,
            previousTarget,
            sim.handle_world
        )

        self.ikSolver:syncFromScene()

        self:abort(string.format(
            'Adaptive progression exhausted at t=%.6f s: %s',
            self.elapsed,
            lastReason
        ))
        return
    end

    if attemptsUsed > 1 and not self.slowdownReported then
        self:message(string.format(
            'Local slowdown: progress=%.6f s; attempts=%d; t=%.6f s',
            nextTime - self.elapsed,
            attemptsUsed,
            nextTime
        ))
    end

    self.slowdownReported = attemptsUsed > 1

        local segmentValid, segmentReason =
            self.modules.SegmentValidation.check(
                self.robot,
                self.validator,
                previousQ,
                candidate,
                config.segmentResolutionRad
            )

        if not segmentValid then
            -- Read back the configuration restored by the segment checker.
            local restoredQ = self.robot:readConfiguration()
            local maxRestoreError = 0

            for i = 1, #previousQ do
                maxRestoreError = math.max(
                    maxRestoreError,
                    math.abs(restoredQ[i] - previousQ[i])
                )
            end

            local restorationPassed = maxRestoreError <= 1e-9

            self:message(string.format(
                'Segment rollback check: %s; max error=%.3e rad; '
                    .. 'accepted progress=%.6f s',
                restorationPassed and 'PASS' or 'FAIL',
                maxRestoreError,
                self.elapsed
            ))

            if not restorationPassed then
                segmentReason = segmentReason .. string.format(
                    ' Segment rollback failed: max error=%.3e rad.',
                    maxRestoreError
                )
            end
            -- The segment checker has already restored the robot.
            sim.setObjectPose(
                self.target,
                previousTarget,
                sim.handle_world
            )

            self.ikSolver:syncFromScene()

            self:abort(
                'Joint segment rejected: ' .. segmentReason
            )
            return
        end

    -- Apply temporarily, then check the resulting scene geometry.
    self.robot:setConfiguration(candidate)

    local valid, reason = self.validator:checkState()
        if not valid then
        self.robot:setConfiguration(previousQ)

        sim.setObjectPose(
            self.target,
            previousTarget,
            sim.handle_world
        )

        -- Diagnostic: read back the scene joints after restoration.
        local restoredQ = self.robot:readConfiguration()
        local maxRestoreError = 0

        for i = 1, #previousQ do
            local difference = math.abs(restoredQ[i] - previousQ[i])
            maxRestoreError = math.max(maxRestoreError, difference)
        end

        local restoreTolerance = 1e-9 -- rad; diagnostic tolerance
        local restorationPassed = maxRestoreError <= restoreTolerance

        self:message(string.format(
            'Joint rollback check: %s; max error=%.3e rad; '
                .. 'tolerance=%.1e rad; accepted progress=%.6f s',
            restorationPassed and 'PASS' or 'FAIL',
            maxRestoreError,
            restoreTolerance,
            self.elapsed
        ))

        if not restorationPassed then
            reason = reason .. string.format(
                ' Joint rollback failed: max error=%.3e rad.',
                maxRestoreError
            )
        end

        self:abort(reason)
        return
    end

    self.elapsed = nextTime
    self.replayTime = self.replayTime + sim.getSimulationTimeStep()
    self:recordAcceptedState(attemptsUsed)

    if self.elapsed >= config.duration then
        self:complete()
    end
end

function Experiment:stepConnectorReplay()
    local sim = self.sim
    local connection = self.connection
    local nextIndex = self.pathIndex + 1

    if nextIndex > #connection.path then
        self:complete()
        return
    end

    local previousQ = self.robot:readConfiguration()
    local previousTarget = sim.getObjectPose(
        self.target, sim.handle_world
    )

    local candidate = connection.path[nextIndex]
    local targetPose = connection.targetPoses[nextIndex]

    -- Restore the last accepted scene state on rejection or Lua error.
    local executed, valid, reason = pcall(function()
        local jointsValid, jointReason =
            self.validator:checkJoints(candidate, previousQ)

        if not jointsValid then
            return false, jointReason
        end

        local segmentValid, segmentReason =
            self.modules.SegmentValidation.check(
                self.robot,
                self.validator,
                previousQ,
                candidate,
                self.config.segmentResolutionRad
            )

        if not segmentValid then
            return false, 'Joint segment rejected: '
                .. tostring(segmentReason)
        end

        sim.setObjectPose(self.target, targetPose, self.anatomy)
        self.robot:setConfiguration(candidate)

        -- Includes tip tracking against the stored reference.
        return self.validator:checkState()
    end)

    if not executed or not valid then
        self.robot:setConfiguration(previousQ)
        sim.setObjectPose(
            self.target, previousTarget, sim.handle_world
        )
        self.ikSolver:syncFromScene()

        local failureReason
        if executed then
            failureReason = tostring(reason)
        else
            failureReason = 'Lua error: ' .. tostring(valid)
        end

        self:abort(string.format(
            'Connector replay sample %d/%d: %s',
            nextIndex - 1,
            #connection.path - 1,
            failureReason
        ))
        return
    end

    self.pathIndex = nextIndex
    self.pathFraction =
        (nextIndex - 1) / (#connection.path - 1)

    self.replayTime =
        self.replayTime + sim.getSimulationTimeStep()

    -- For discrete replay, progress time is accepted replay time.
    self.elapsed = self.replayTime

    -- No IK solve takes place during replay.
    self:recordAcceptedState(0)

    if nextIndex == #connection.path then
        self:complete()
    end
end

function Experiment:complete()
    self.running = false
    local p = self.sim.getObjectPosition(self.tip, self.anatomy)

    local finalError = norm({
        p[1] - self.goalPosition[1],
        p[2] - self.goalPosition[2],
        p[3] - self.goalPosition[3],
    })

    self.finalTipError = finalError

    if finalError ~= finalError
        or finalError == math.huge
        or finalError > self.config.maxTipError then

        self:abort(string.format(
            'Final tip-to-%s error rejected: %.6f mm',
            self.config.targetName,
            1000 * finalError
        ))
        return
    end

    self:message(string.format(
        'TEST COMPLETED: %d accepted configurations recorded.',
        self.recorder.count
    ))

    self:message(string.format(
        'Maximum RCM error: %.4f mm',
        1000*self.recorder.maxRcmError
    ))

    self:message(string.format(
        'Maximum tip tracking error: %.4f mm',
        1000*self.recorder.maxTipError
    ))

    self:message(string.format(
        'Minimum shaft-wall clearance: %.4f mm',
        1000*self.recorder.minClearance
    ))

    if self.recorder.minObstacleClearance ~= nil then
        self:message(string.format(
            'Minimum shaft-obstacle clearance: %.4f mm',
            1000 * self.recorder.minObstacleClearance
        ))
    end

    local finalError = norm({
        p[1] - self.goalPosition[1],
        p[2] - self.goalPosition[2],
        p[3] - self.goalPosition[3]
    })

    self:message(string.format(
        'Final tip-to-%s error: %.4f mm',
        self.config.targetName,
        1000*finalError
    ))

    self:message(string.format(
        'Recorded tip path length: %.4f mm.',
        1000 * self.recorder.tipPathLength
    ))

    self:message(string.format(
        'Recorded reference path length: %.4f mm.',
        1000 * self.recorder.referencePathLength
    ))

    self.status = 'completed'
    self.reason = 'Target reached; sampled checks passed'
end

function Experiment:exportRun()
    assert(self.recorder ~= nil, 'Export recorder was not initialized')

    self.recorder:write({
        startedAt = self.startedAt,
        status = self.status,
        reason = self.reason,
        replayTime = self.replayTime,
        progressTime = self.elapsed,
        initialAnatomyPose = self.initialAnatomyPose,
        startPosition = self.startPosition,
        goalPosition = self.goalPosition,
        executionMode = self.config.executionMode,
        replayStarted = self.replayStarted == true,
        pathReplayed = self.config.executionMode ~= 'prescribed_motion' and self.status == 'completed',
        finalTipError = self.finalTipError,
        plannerStats = self.plannerStats,
        obstaclePose = self.initialObstaclePose,
        obstacleDimensions = self.obstacleDimensions,
        planningWallTime = self.planningWallTime,
        planningProfile = self.planningProfile,
        plannerCostHistory = self.plannerCostHistory,
    })
end

function Experiment:cleanup()
    local sim = self.sim
    self.running = false

    if self.ikSolver then
        self.ikSolver:destroy()
    end

    if self.robotCollection then
        sim.destroyCollection(self.robotCollection)
    end

    if self.savedTargetPose then
        sim.setObjectPose(
            self.target,
            self.savedTargetPose,
            sim.handle_world
        )
    end

    if self.savedDynamics ~= nil then
        sim.setBoolParam(
            sim.boolparam_dynamics_handling_enabled,
            self.savedDynamics
        )
    end

    if self.diagnosticOnly then
        self:message('Static diagnostic finished; no planning/replay CSV exported.')
        return
    end

    local ok, err = pcall(function()
        self:exportRun()
    end)

    if not ok then
        sim.addLog(
            sim.verbosity_errors,
            'EXPORT INCOMPLETE: ' .. tostring(err)
        )
    end
end

return Experiment