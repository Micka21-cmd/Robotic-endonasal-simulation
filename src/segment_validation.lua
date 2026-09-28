-- Discrete checks along a straight joint-space segment.
-- Temporarily moves the scene robot, then restores its entry configuration.
-- Does not move the target, run IK, or record experimental samples.

local SegmentValidation = {}

local function isFinite(value)
    return type(value) == 'number'
        and value == value
        and value > -math.huge
        and value < math.huge
end

function SegmentValidation.check(
    robot, validator, startQ, goalQ, resolution
)
    assert(
        isFinite(resolution) and resolution > 0,
        'Segment resolution must be finite and positive'
    )

    local jointCount = #robot.joints

    assert(
        #startQ == jointCount and #goalQ == jointCount,
        'Segment configurations must match the robot joint count'
    )

    local maxDelta = 0

    for i = 1, jointCount do
        assert(
            isFinite(startQ[i]) and isFinite(goalQ[i]),
            'Segment joint values must be finite'
        )

        maxDelta = math.max(
            maxDelta,
            math.abs(goalQ[i] - startQ[i])
        )
    end

    -- At least two intervals: always inspect a midpoint.
    local intervals = math.max(
        2,
        math.ceil(maxDelta / resolution)
    )

    local originalQ = robot:readConfiguration()

    -- Protected execution ensures restoration is attempted
    -- even if a measurement raises a Lua error.
    local executed, valid, reason = pcall(function()
        local previousSample = startQ

        for sample = 0, intervals do
            local fraction = sample / intervals
            local q = {}

            for i = 1, jointCount do
                q[i] = startQ[i]
                    + fraction * (goalQ[i] - startQ[i])
            end

            local jointsValid, jointReason =
                validator:checkJoints(q, previousSample)

            if not jointsValid then
                return false, string.format(
                    'Segment sample %d/%d: %s',
                    sample, intervals, jointReason
                )
            end

            robot:setConfiguration(q)

            local geometryValid, geometryReason =
                validator:checkState(false)

            if not geometryValid then
                return false, string.format(
                    'Segment sample %d/%d: %s',
                    sample, intervals, geometryReason
                )
            end

            previousSample = q
        end

        return true
    end)

    robot:setConfiguration(originalQ)

    if not executed then
        error(valid, 0)
    end

    return valid, reason
end

return SegmentValidation