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

local exportDir = config.exportDir
local runLabel = config.runLabel


local dataRows = {}
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

local worstRcm = 0
local worstTip = 0
local smallestClearance = math.huge
local checks = 0

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

    assert(clearance ~= nil, 'Distance indisponible pour export')

    local row = {
        #dataRows,             -- indice : 0 pour le depart
        replayTime,            -- temps cumule des pas de relecture
        elapsed,               -- progression sur la loi nominale
        elapsed / duration,    -- tau
        attempts
    }

    for i = 1, 6 do
        row[#row + 1] = q[i]
    end
    for i = 1, 3 do
        row[#row + 1] = p[i]
    end
    for i = 1, 3 do
        row[#row + 1] = pd[i]
    end

    row[#row + 1] = rcm
    row[#row + 1] = tipError
    row[#row + 1] = clearance
    row[#row + 1] = collision and 1 or 0

    dataRows[#dataRows + 1] = row

    -- Update statistics only after recording an accepted state.
    checks = checks + 1
    worstRcm = math.max(worstRcm, rcm)
    worstTip = math.max(worstTip, tipError)
    smallestClearance = math.min(smallestClearance, clearance)
end

local function csvCell(value)
    if type(value) == 'number' then
        -- Point decimal, precision suffisante pour les petites erreurs.
        return (string.format('%.15g', value):gsub(',', '.'))
    end

    local text = tostring(value)
    return '"' .. text:gsub('"', '""') .. '"'
end

local function writeCsv(path, header, rows)
    local file, err = io.open(path, 'w')
    if not file then
        error('Ouverture impossible : ' .. path .. ' : ' .. tostring(err))
    end

    local ok, writeError = pcall(function()
        local function writeRow(row)
            local cells = {}
            for i, value in ipairs(row) do
                cells[i] = csvCell(value)
            end
            assert(file:write(table.concat(cells, ',') .. '\n'))
        end

        writeRow(header)
        for _, row in ipairs(rows) do
            writeRow(row)
        end
    end)

    local closed, closeError = file:close()
    if not ok then error(writeError) end
    if not closed then error(closeError) end
end

local function fileExists(path)
    local file = io.open(path, 'r')
    if file then
        file:close()
        return true
    end
    return false
end

local function exportRun()
    local runDir, runId = RunDirectory.create(
        exportDir,
        runLabel,
        runStartedAt or os.time()
    )

    local stem = runDir .. '/' .. runLabel

    local header = {
        'sample', 'replay_time_s', 'progress_time_s', 'tau',
        'ik_attempts',
        'q1_rad', 'q2_rad', 'q3_rad',
        'q4_rad', 'q5_rad', 'q6_rad',
        'tip_x_m', 'tip_y_m', 'tip_z_m',
        'target_x_m', 'target_y_m', 'target_z_m',
        'rcm_error_m', 'tip_error_m', 'clearance_m',
        'robot_wall_collision'
    }

    writeCsv(stem .. '_states.csv', header, dataRows)

    local summary = {
        {'run_label', runLabel},
        {'status', runStatus},
        {'reason', runReason},
        {'accepted_samples', #dataRows},
        {'nominal_duration_s', duration},
        {'replay_time_s', replayTime},
        {'progress_time_s', elapsed},
        {'max_rcm_error_allowed_m', maxRcmError},
        {'max_tip_error_allowed_m', maxTipError},
        {'min_clearance_allowed_m', minClearance},
        {'max_joint_step_rad', maxJointStep},
        {'position_frame', 'Anatomy_V0'},
        {'dynamics_enabled_during_tracking', 0},
        {'run_id', runId},
        {'started_at_local', os.date(
            '%Y-%m-%dT%H:%M:%S', runStartedAt or os.time()
        )},
        {'method', config.method},
        {'run_note', config.runNote},
        {'ik_damping', config.ikDamping},
        {'ik_max_iterations', config.ikMaxIterations},
        {'ik_position_tolerance_m', config.ikPositionTolerance},
        {'ik_orientation_tolerance_rad', config.ikOrientationTolerance},
        {'max_ik_attempts', config.maxIkAttempts},
    }
    summary[#summary + 1] = {'target_name', config.targetName}
    summary[#summary + 1] = {'target_path', config.targetPath}

    if startPosition and goalPosition then
        local axes = {'x', 'y', 'z'}

        for i = 1, 3 do
            summary[#summary + 1] = {
                'start_' .. axes[i] .. '_m',
                startPosition[i]
            }

            summary[#summary + 1] = {
                'goal_' .. axes[i] .. '_m',
                goalPosition[i]
            }
        end
    end

    -- Ne pas publier de faux extrema si aucun etat n'a ete accepte.
    if #dataRows > 0 then
        summary[#summary + 1] = {'max_rcm_error_m', worstRcm}
        summary[#summary + 1] = {'max_tip_error_m', worstTip}
        summary[#summary + 1] = {
            'min_clearance_m', smallestClearance
        }
    end

    if initialAnatomyPose then
        local names = {'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw'}
        for i = 1, 7 do
            summary[#summary + 1] = {
                'anatomy_world_' .. names[i],
                initialAnatomyPose[i]
            }
        end
    end

    writeCsv(stem .. '_summary.csv', {'parameter', 'value'}, summary)

    message('Export etats : ' .. stem .. '_states.csv')
    message('Export bilan : ' .. stem .. '_summary.csv')
    message(string.format(
        'Export termine : %d configurations ; statut=%s',
        #dataRows, runStatus
    ))
end

function sysCall_init()
    local robot = sim.getObject('..')
    runStartedAt = os.time()
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
            'TEST TERMINE : %d configurations controlees.', checks
        ))
        message(string.format(
            'Erreur RCM maximale : %.4f mm', 1000*worstRcm
        ))
        message(string.format(
            'Erreur de suivi maximale : %.4f mm', 1000*worstTip
        ))
        message(string.format(
            'Distance minimale tige-parois : %.4f mm',
            1000*smallestClearance
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