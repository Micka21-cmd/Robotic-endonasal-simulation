-- Configuration for the V0 prescribed-motion experiment.
-- Distances are expressed in metres, angles in radians and times in seconds.

local Config = {}

function Config.create(projectRoot)
    return {
        exportDir = projectRoot .. '/Results',
        runLabel = 'V0_B_T1',

        -- Nominal trajectory duration, excluding adaptive slowdown.
        duration = 16,

        -- Numerical acceptance thresholds; not clinical specifications.
        maxRcmError = 0.0001,
        maxTipError = 0.0005,
        minClearance = 0.001,

        -- Maximum joint increment between accepted configurations.
        -- This is not a joint velocity limit.
        maxJointStep = math.rad(0.5),

        -- Adaptive progression settings.
        maxIkAttempts = 8,

        -- Internal IK solver settings.
        ikDamping = 0.01,
        ikMaxIterations = 100,
        ikPositionTolerance = 0.00001,
        ikOrientationTolerance = math.rad(0.01),
    }
end

return Config