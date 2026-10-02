-- Local Cartesian connection under the existing RCM convention.
-- Returns sampled joint configurations, without replaying the path.
-- Restores robot, target and IK synchronization before returning.

local LocalConnector = {}
LocalConnector.__index = LocalConnector

local function copy(values)
    local result = {}
    for i, value in ipairs(values) do
        result[i] = value
    end
    return result
end

local function isFinite(value)
    return type(value) == 'number'
        and value == value
        and value > -math.huge
        and value < math.huge
end

function LocalConnector.new(
    simApi, robot, ikSolver, validator, modules, objects, config
)
    return setmetatable({
        sim = simApi,
        robot = robot,
        ikSolver = ikSolver,
        validator = validator,
        modules = modules,
        objects = objects,
        config = config,
    }, LocalConnector)
end

function LocalConnector:connect(startQ, goalPosition)
    local sim = self.sim
    local config = self.config
    local objects = self.objects

    assert(#startQ == #self.robot.joints,
        'Invalid start configuration size')

    for i = 1, #self.robot.joints do
        assert(isFinite(startQ[i]),
            'Start joint values must be finite')
    end

    assert(#goalPosition == 3, 'Goal must contain three coordinates')
    for i = 1, 3 do
        assert(isFinite(goalPosition[i]),
            'Goal coordinates must be finite')
    end
    assert(goalPosition[3] > 0,
        'Goal must have positive anatomical Z')

    local step = config.localConnectorStepM
    local maxSteps = config.localConnectorMaxSteps

    assert(isFinite(step) and step > 0,
        'Connector step must be finite and positive')
    assert(
        isFinite(maxSteps) and maxSteps >= 1
            and maxSteps == math.floor(maxSteps),
        'Connector step limit must be a positive integer'
    )

    local originalQ = self.robot:readConfiguration()
    local originalTarget = sim.getObjectPose(
        objects.target, sim.handle_world
    )

    local executed, result = pcall(function()
        local jointsValid, jointReason =
            self.validator:checkJoints(startQ, startQ)

        if not jointsValid then
            return {success = false, reason = jointReason}
        end

        self.robot:setConfiguration(startQ)

        local startValid, startReason =
            self.validator:checkState(false)

        if not startValid then
            return {success = false, reason = startReason}
        end

        local startPosition = sim.getObjectPosition(
            objects.tip, objects.anatomy
        )

        assert(startPosition[3] > 0,
            'Start tip must have positive anatomical Z')

        local squaredDistance = 0
        for i = 1, 3 do
            squaredDistance = squaredDistance
                + (goalPosition[i] - startPosition[i])^2
        end

        local intervals = math.max(
            1,
            math.ceil(math.sqrt(squaredDistance) / step)
        )

        if intervals > maxSteps then
            return {
                success = false,
                reason = 'Connector step budget exceeded.',
            }
        end

        local path = {copy(startQ)}
        -- The initial reference is the actual entry tip pose.
        -- Subsequent references are the Cartesian commands used by IK.
        local targetPoses = {
            copy(sim.getObjectPose(objects.tip, objects.anatomy))
        }
        local previousQ = copy(startQ)

        for sample = 1, intervals do
            local position = {}
            local fraction = sample / intervals

            for i = 1, 3 do
                position[i] = startPosition[i]
                    + fraction * (goalPosition[i] - startPosition[i])
            end

            -- Reuse the existing RCM-compatible pose convention.
            -- Identical endpoints make poseAt return this position.
            local pose = self.modules.Trajectory.poseAt(
                1, 1, position, position
            )

            sim.setObjectPose(objects.target, pose, objects.anatomy)

            local candidate, flags = self.ikSolver:solveCandidate()

            if candidate == nil then
                return {
                    success = false,
                    reason = string.format(
                        'Connector sample %d/%d: IK failed, flags=%s',
                        sample, intervals, tostring(flags)
                    ),
                }
            end

            local valid, reason =
                self.validator:checkJoints(candidate, previousQ)

            if valid then
                valid, reason = self.modules.SegmentValidation.check(
                    self.robot,
                    self.validator,
                    previousQ,
                    candidate,
                    config.segmentResolutionRad
                )
            end

            if valid then
                self.robot:setConfiguration(candidate)
                valid, reason = self.validator:checkState()
            end

            if not valid then
                return {
                    success = false,
                    reason = string.format(
                        'Connector sample %d/%d: %s',
                        sample, intervals, reason
                    ),
                }
            end

            path[#path + 1] = copy(candidate)
            targetPoses[#targetPoses + 1] = copy(pose)
            previousQ = copy(candidate)
        end

        return {
            success = true,
            path = path,
            targetPoses = targetPoses,
            reason = 'Local connection passed sampled checks.',
        }
    end)

    -- Restore the caller's scene, including after a Lua error.
    self.robot:setConfiguration(originalQ)
    sim.setObjectPose(
        objects.target, originalTarget, sim.handle_world
    )
    self.ikSolver:syncFromScene()

    if not executed then
        error(result, 0)
    end

    return result
end

return LocalConnector