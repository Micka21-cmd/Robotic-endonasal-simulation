-- Experiment settings.
-- Distances in metres, angles in radians, times in seconds.

local Config = {}

function Config.create(projectRoot)
    return {
        exportDir = projectRoot .. '/Results',
        runLabel = 'V0_B_T1',
        method = 'prescribed_motion_ik',
        runNote = 'T0 to T1 verification after trajectory module extraction',

        duration = 16,

        maxRcmError = 0.0001,
        maxTipError = 0.0005,
        minClearance = 0.001,
        maxJointStep = math.rad(0.5),

        maxIkAttempts = 8,

        ikDamping = 0.01,
        ikMaxIterations = 100,
        ikPositionTolerance = 0.00001,
        ikOrientationTolerance = math.rad(0.01),
    }
end

return Config