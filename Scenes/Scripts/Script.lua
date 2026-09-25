sim = require 'sim'
simIK = require 'simIK'

-- Local checkout location. Adapt this path when moving the repository.
local projectRoot =
    'C:/Users/Micka/Documents/GitHub/Robotic-endonasal-simulation'

local Config = dofile(projectRoot .. '/src/config.lua')
local config = Config.create(projectRoot)

local Robot = dofile(projectRoot .. '/src/robot.lua')
local robotModel

local Kinematics = dofile(projectRoot .. '/src/kinematics.lua')
local ikSolver

local RunDirectory = dofile(projectRoot .. '/src/run_directory.lua')
local runStartedAt

local Trajectory = dofile(projectRoot .. '/src/trajectory.lua')

local Validation = dofile(projectRoot .. '/src/validation.lua')
local validator

local Export = dofile(projectRoot .. '/src/export.lua')
local recorder


local replayTime = 0
local runStatus = 'incomplete'
local runReason = 'Stopped before completion, or script error'
local initialAnatomyPose

local joints = {}
local anatomy, shaft, tip, target, wall, robotCollection
local savedDynamics, savedTargetPose
local running = false
local elapsed = 0

local duration = config.duration
local maxRcmError = config.maxRcmError
local maxTipError = config.maxTipError
local minClearance = config.minClearance
local maxJointStep = config.maxJointStep


local goal
local startPosition
local goalPosition

local slowdownReported = false

local function message(text)
    sim.addLog(sim.verbosity_scriptinfos, text)
end

local function norm(v)
    return math.sqrt(v[1]^2 + v[2]^2 + v[3]^2)
end


local function abortTest(reason)
    running = false
    runStatus = 'failed'
    runReason = reason

    sim.addLog(
        sim.verbosity_errors,
        'TEST INTERROMPU : ' .. reason
    )
    sim.pauseSimulation()
end

local function measureState()
    -- Dans le repere de la tige, son axe est l'axe Z.
    local entry = sim.getObjectPosition(anatomy, shaft)
    local _, _, dimensions = sim.getShapeGeomInfo(shaft)
    local halfLength = dimensions[3] / 2

    local rcmError = math.sqrt(entry[1]^2 + entry[2]^2)
    local entryOnShaft = math.abs(entry[3]) <= halfLength

    local tipError = norm(sim.getObjectPosition(tip, target))
    local collision = sim.checkCollision(robotCollection, wall) > 0

    local result, data = sim.checkDistance(shaft, wall, 0)
    local clearance = nil
    if result > 0 then
        clearance = data[7]
    end

    return rcmError, tipError, clearance, entryOnShaft, collision
end

local function validateState()
    local rcm, err, distance, onShaft, collision = measureState()

    if not onShaft then
        return false, 'le RCM est hors de la longueur de la tige.'
    end
    if collision then
        return false, 'collision robot-corridor.'
    end
    if distance == nil then
        return false, 'distance tige-parois indisponible.'
    end
    if rcm > maxRcmError then
        return false, string.format('erreur RCM = %.4f mm.', 1000*rcm)
    end
    if err > maxTipError then
        return false, string.format('erreur pointe = %.4f mm.', 1000*err)
    end
    if distance < minClearance then
        return false, string.format('marge = %.4f mm.', 1000*distance)
    end

    return true
end


local function recordAcceptedState(attempts)
    local q = robotModel:readConfiguration()
    local p = sim.getObjectPosition(tip, anatomy)
    local pd = sim.getObjectPosition(target, anatomy)

    local rcm, tipError, clearance, _, collision = measureState()

    recorder:record({
        replayTime = replayTime,
        progressTime = elapsed,
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


local function exportRun()
    assert(recorder ~= nil, 'Export recorder was not initialized')

    recorder:write({
        startedAt = runStartedAt,
        status = runStatus,
        reason = runReason,
        replayTime = replayTime,
        progressTime = elapsed,
        initialAnatomyPose = initialAnatomyPose,
        startPosition = startPosition,
        goalPosition = goalPosition
    })
end

function sysCall_init()
    local robot = sim.getObject('..')
    runStartedAt = os.time()
    recorder = Export.new(config, RunDirectory, message)
    anatomy = sim.getObject('/Anatomy_V0')
    shaft = sim.getObject('/UR3/Instrument_shaft')
    tip = sim.getObject('/UR3/tool_tip')
    target = sim.getObject('/tool_target')
    wall = sim.getObject('/Anatomy_V0/Corridor_wall')
    goal = sim.getObject(config.targetPath)

    startPosition = {
        config.startPosition[1],
        config.startPosition[2],
        config.startPosition[3]
    }

    goalPosition = sim.getObjectPosition(goal, anatomy)

    assert(
        startPosition[3] > 0 and goalPosition[3] > 0,
        'Start and goal must have positive anatomical Z'
    )
    initialAnatomyPose = sim.getObjectPose(anatomy, sim.handle_world)

    robotModel = Robot.new(sim, robot, '../joint', 6)

    -- Shared joint handles used by the existing IK setup.
    joints = robotModel.joints

    savedDynamics = sim.getBoolParam(
        sim.boolparam_dynamics_handling_enabled
    )
    savedTargetPose = sim.getObjectPose(target, sim.handle_world)
    sim.setBoolParam(sim.boolparam_dynamics_handling_enabled, false)

    -- Controle du depart attendu : pointe en T0.
    local p = sim.getObjectPosition(tip, anatomy)
    local startError = norm({
        p[1] - startPosition[1],
        p[2] - startPosition[2],
        p[3] - startPosition[3]
    })

    if startError > 0.0001 then
        abortTest('Tool tip is not at T0 at initialization.')
        return
    end

    sim.setObjectPose(target, sim.getObjectPose(tip, sim.handle_world),
                      sim.handle_world)

    robotCollection = sim.createCollection(1)
    sim.addItemToCollection(robotCollection, sim.handle_tree, robot, 0)

    validator = Validation.new(
        sim,
        robotModel,
        {
            anatomy = anatomy,
            shaft = shaft,
            tip = tip,
            target = target,
            wall = wall,
            robotCollection = robotCollection
        },
        config
    )

    ikSolver = Kinematics.new(simIK, robotModel, config)
    ikSolver:initialize(tip, target)

    ikSolver = Kinematics.new(simIK, robotModel, config)
    ikSolver:initialize(tip, target)

    local valid, reason = validateState()
    if not valid then
        abortTest(reason)
        return
    end


    running = true
    recordAcceptedState(0)
    message(string.format(
        'Valid start. Kinematic RCM test: T0 -> %s; nominal duration %.1f s.',
        config.targetName,
        duration
    ))
end

function sysCall_actuation()
    if not running then return end

    local previousQ = robotModel:readConfiguration()
    local previousTarget = sim.getObjectPose(target, sim.handle_world)

    local advance = sim.getSimulationTimeStep()
    local nextTime
    local candidate
    local accepted = false
    local lastReason = ''
    local attemptsUsed = 0

    -- Jusqu'a 8 essais, en divisant la progression par deux.
    -- Le robot reste dans previousQ pendant ces calculs.
    for attempt = 1, config.maxIkAttempts do
        attemptsUsed = attempt
        nextTime = math.min(duration, elapsed + advance)

        local targetPose = Trajectory.poseAt(
            nextTime,
            duration,
            startPosition,
            goalPosition
        )
        sim.setObjectPose(target, targetPose, anatomy)

        -- Repartir de la configuration du robot, toujours inchangee,
        -- y compris apres un essai IK rejete.
        local flags
        candidate, flags = ikSolver:solveCandidate()

            if candidate ~= nil then
                local acceptable, reason =
                validator:checkJoints(candidate, previousQ)

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
        sim.setObjectPose(target, previousTarget, sim.handle_world)
        ikSolver:syncFromScene()

        abortTest(string.format(
            'progression adaptative epuisee a t=%.6f s : %s',
            elapsed, lastReason
        ))
        return
    end

    -- Signaler uniquement l'entree dans une zone de ralentissement.
    if attemptsUsed > 1 and not slowdownReported then
        message(string.format(
            'Ralentissement local : progression=%.6f s ; essais=%d ; t=%.6f s',
            nextTime - elapsed, attemptsUsed, nextTime
        ))
    end
    slowdownReported = attemptsUsed > 1


    -- Apply the candidate temporarily to check its geometry.
    robotModel:setConfiguration(candidate)

    local valid, reason = validateState()

    if not valid then
        robotModel:setConfiguration(previousQ)
        sim.setObjectPose(target, previousTarget, sim.handle_world)
        abortTest(reason)
        return
    end

    elapsed = nextTime
    replayTime = replayTime + sim.getSimulationTimeStep()
    recordAcceptedState(attemptsUsed)

    if elapsed >= duration then
        running = false
        message(string.format(
            'TEST COMPLETED: %d accepted configurations recorded.',
            recorder.count
        ))

        message(string.format(
            'Maximum RCM error: %.4f mm',
            1000*recorder.maxRcmError
        ))

        message(string.format(
            'Maximum tip tracking error: %.4f mm',
            1000*recorder.maxTipError
        ))

        message(string.format(
            'Minimum shaft-wall clearance: %.4f mm',
            1000*recorder.minClearance
        ))

        local finalPosition = sim.getObjectPosition(tip, anatomy)
        local finalError = norm({
            finalPosition[1] - goalPosition[1],
            finalPosition[2] - goalPosition[2],
            finalPosition[3] - goalPosition[3]
        })

        message(string.format(
            'Final tip-to-%s error: %.4f mm',
            config.targetName,
            1000*finalError
        ))
        runStatus = 'completed'
        runReason = 'Target reached; sampled checks passed'
    end
end

function sysCall_cleanup()
    if ikSolver then
        ikSolver:destroy()
    end
    if robotCollection then
        sim.destroyCollection(robotCollection)
    end
    if savedTargetPose then
        sim.setObjectPose(target, savedTargetPose, sim.handle_world)
    end
    if savedDynamics ~= nil then
        sim.setBoolParam(
            sim.boolparam_dynamics_handling_enabled, savedDynamics
        )
    end
    local ok, err = pcall(exportRun)
    if not ok then
        sim.addLog(
            sim.verbosity_errors,
            'EXPORT INCOMPLET : ' .. tostring(err)
        )
    end
end