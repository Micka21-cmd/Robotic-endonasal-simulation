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

function Experiment:initialize()
    local sim = self.sim
    local config = self.config

    assert(
        config.executionMode == 'prescribed_motion'
            or config.executionMode == 'connector_replay'
            or config.executionMode == 'rrt_replay',
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

    if config.executionMode == 'rrt_replay' then
        self:message(string.format(
            'RRT planning: seed=%d; iteration budget=%d.',
            config.rrtSeed,
            config.rrtMaxIterations
        ))

        local startPose = sim.getObjectPose(self.tip, self.anatomy)

        local function plan()
            return self.modules.RRT.plan(
                beforeQ,
                startPose,
                self.goalPosition,
                self.localConnector,
                config
            )
        end

        local planningStartedAt = sim.getSystemTime()

        if config.profilePlanning then
            connection = self:profilePlanning(plan)
        else
            connection = plan()
        end

        self.planningWallTime =
            sim.getSystemTime() - planningStartedAt

        self:message(string.format(
            'RRT planning wall time: %.6f s; profiling=%s.',
            self.planningWallTime,
            config.profilePlanning and 'on' or 'off'
        ))

        if self.planningProfile then
            for _, row in ipairs(self.planningProfile) do
                self:message(string.format(
                    'PROFILE %s: calls=%d; inclusive=%.6f s; '
                        .. 'exclusive=%.6f s; Lua errors=%d.',
                    row.name,
                    row.calls,
                    row.inclusive,
                    row.exclusive,
                    row.errors
                ))
            end
        end
        self.plannerStats = connection.stats

        self:message(string.format(
            'RRT result: %s; iterations=%d; nodes=%d; '
                .. 'connector calls=%d; rejected connections=%d.',
            connection.success and 'SUCCESS' or 'FAILED',
            connection.stats.iterations,
            connection.stats.nodes,
            connection.stats.connector_calls,
            connection.stats.rejected_connections
        ))
    else
        connection = self.localConnector:connect(
            beforeQ,
            self.goalPosition
        )
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