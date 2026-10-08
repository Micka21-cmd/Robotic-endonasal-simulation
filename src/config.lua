-- Experiment settings.
-- Distances in metres, angles in radians, times in seconds.

local Config = {}

function Config.create(projectRoot)
    local targetName = 'T2'
    local executionMode = 'rrt_star_replay'

    assert(
        targetName == 'T1' or targetName == 'T2',
        'Unsupported target name'
    )
    assert(
        executionMode == 'prescribed_motion'
            or executionMode == 'connector_replay'
            or executionMode == 'rrt_replay'
            or executionMode == 'rrt_star_replay',
        'Unsupported execution mode'
    )
    return {
        exportDir = projectRoot .. '/Results',

        targetName = targetName,
        targetPath = '/Anatomy_V0/target_' .. targetName,
        startPosition = {0, 0, 0.075},

        runLabel = 'V0_B_O1_' .. targetName,

        method = ({
            prescribed_motion = 'prescribed_motion_ik',
            connector_replay = 'local_connector_replay',
            rrt_replay = 'rrt_task_space_ik',
            rrt_star_replay = 'rrt_star_task_space_ik',
        })[executionMode],

        runNote = 'O1 RRT-star budget extension; seed 42; 50 iterations',

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

        -- RRT sampling bounds in the anatomical frame, metres.
        rrtBoundsMin = {-0.004, -0.004, 0.073},
        rrtBoundsMax = { 0.004,  0.004, 0.077},

        rrtExtensionM = 0.001,
        rrtGoalBias = 0.15,
        rrtMaxIterations = 50,
        rrtSeed = 42,
        rrtStarRadiusMaxM = 0.0015, -- rayon maximal : 1,5 mm
        rrtStarGammaM = 0.003,     -- coefficient du rayon décroissant : 3 mm

        obstacleEnabled = true,
        obstaclePath = '/Anatomy_V0/Obstacle_O1',
        minObstacleClearance = 0.001,
        executionMode = executionMode,

        profilePlanning = true,
    }
end

return Config