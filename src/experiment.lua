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

function Experiment:initialize()
    local sim = self.sim
    local config = self.config

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

    local connection = self.localConnector:connect(
        beforeQ,
        self.goalPosition
    )

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
            'Local connector diagnostic failed: '
                .. tostring(connection.reason)
        )
        return
    end

    self:message(string.format(
        'Local connector T0 -> %s: PASS; %d configurations returned.',
        config.targetName,
        #connection.path
    ))

    self:recordAcceptedState(0)
    self.running = true

    self:message(string.format(
        'Valid start. Kinematic RCM test: T0 -> %s; nominal duration %.1f s.',
        config.targetName,
        config.duration
    ))
end

function Experiment:recordAcceptedState(attempts)
    local q = self.robot:readConfiguration()
    local p = self.sim.getObjectPosition(self.tip, self.anatomy)
    local pd = self.sim.getObjectPosition(self.target, self.anatomy)

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
        collision = collision
    })
end

function Experiment:step()
    if not self.running then
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

function Experiment:complete()
    self.running = false

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

    local p = self.sim.getObjectPosition(self.tip, self.anatomy)
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
        goalPosition = self.goalPosition
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