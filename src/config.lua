-- Experiment settings.
-- Distances in metres, angles in radians, times in seconds.

local Config = {}

function Config.create(projectRoot)
    local targetName = 'T2'

    assert(
        targetName == 'T1' or targetName == 'T2',
        'Unsupported target name'
    )

    return {
        exportDir = projectRoot .. '/Results',

        targetName = targetName,
        targetPath = '/Anatomy_V0/target_' .. targetName,
        startPosition = {0, 0, 0.075},

        runLabel = 'V0_B_' .. targetName,
        method = 'prescribed_motion_ik',
        runNote = 'T2 prescribed-motion test with sampled joint-segment checks',

        duration = 16,

        maxRcmError = 0.0001,
        maxTipError = 0.0005,
        minClearance = 0.001,
        maxJointStep = math.rad(0.5),

        segmentResolutionRad = math.rad(0.1),

        maxIkAttempts = 8,

        localConnectorStepM = 0.0001, -- 0.1 mm between tip commands
        localConnectorMaxSteps = 1000,

        ikDamping = 0.01,
        ikMaxIterations = 100,
        ikPositionTolerance = 0.00001,
        ikOrientationTolerance = math.rad(0.01),
    }
end

return Config