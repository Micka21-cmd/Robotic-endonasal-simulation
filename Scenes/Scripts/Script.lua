sim = require 'sim'
simIK = require 'simIK'

-- Local checkout location.
local projectRoot =
    'C:/Users/Micka/Documents/GitHub/Robotic-endonasal-simulation'

local function loadModule(name)
    return dofile(projectRoot .. '/src/' .. name .. '.lua')
end

local Config = loadModule('config')
local Experiment = loadModule('experiment')

local modules = {
    Robot = loadModule('robot'),
    Kinematics = loadModule('kinematics'),
    Trajectory = loadModule('trajectory'),
    Validation = loadModule('validation'),
    RunDirectory = loadModule('run_directory'),
    Export = loadModule('export'),
    SegmentValidation = loadModule('segment_validation')
}

local experiment

function sysCall_init()
    local config = Config.create(projectRoot)

    experiment = Experiment.new(sim, simIK, config, modules)
    experiment:initialize()
end

function sysCall_actuation()
    if experiment then
        experiment:step()
    end
end

function sysCall_cleanup()
    if experiment then
        experiment:cleanup()
    end
end