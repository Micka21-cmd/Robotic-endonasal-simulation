-- Discrete joint and geometric checks for the current experiment.
-- Validation does not move the robot or update experiment statistics.

local Validation = {}
Validation.__index = Validation

local function norm(vector)
    return math.sqrt(
        vector[1]^2 + vector[2]^2 + vector[3]^2
    )
end

function Validation.new(simApi, robot, objects, config)
    local self = setmetatable({}, Validation)

    self.sim = simApi
    self.robot = robot
    self.objects = objects
    self.config = config

    return self
end

function Validation:checkJoints(candidate, previous)
    for i, joint in ipairs(self.robot.joints) do
        local cyclic, interval = self.sim.getJointInterval(joint)

        local withinLimits = cyclic or (
            candidate[i] >= interval[1] - 1e-9 and
            candidate[i] <= interval[1] + interval[2] + 1e-9
        )

        -- Preserve the original direct angle-difference convention.
        local delta = math.abs(candidate[i] - previous[i])

        if not withinLimits then
            return false, string.format(
                'Joint limit exceeded at J%d', i
            )
        end

        if delta > self.config.maxJointStep then
            return false, string.format(
                'Joint increment at J%d = %.6f deg',
                i,
                math.deg(delta)
            )
        end
    end

    return true
end

function Validation:measureState()
    local sim = self.sim
    local objects = self.objects

    -- The shaft's longitudinal axis is its local Z axis.
    local entry = sim.getObjectPosition(
        objects.anatomy,
        objects.shaft
    )

    local _, _, dimensions = sim.getShapeGeomInfo(objects.shaft)
    local halfLength = dimensions[3] / 2

    local rcmError = math.sqrt(entry[1]^2 + entry[2]^2)
    local entryOnShaft = math.abs(entry[3]) <= halfLength

    local tipError = norm(sim.getObjectPosition(
        objects.tip,
        objects.target
    ))

    local collision = sim.checkCollision(
        objects.robotCollection,
        objects.wall
    ) > 0

    local result, data = sim.checkDistance(
        objects.shaft,
        objects.wall,
        0
    )

    local clearance = nil
    if result > 0 then
        clearance = data[7]
    end

    return rcmError, tipError, clearance, entryOnShaft, collision
end

function Validation:checkState()
    local rcm, tipError, clearance, onShaft, collision =
        self:measureState()

    if not onShaft then
        return false, 'RCM lies outside the physical shaft length.'
    end

    if collision then
        return false, 'Robot-corridor collision.'
    end

    if clearance == nil then
        return false, 'Shaft-wall distance is unavailable.'
    end

    if rcm > self.config.maxRcmError then
        return false, string.format(
            'RCM error = %.4f mm.', 1000*rcm
        )
    end

    if tipError > self.config.maxTipError then
        return false, string.format(
            'Tip tracking error = %.4f mm.', 1000*tipError
        )
    end

    if clearance < self.config.minClearance then
        return false, string.format(
            'Shaft-wall clearance = %.4f mm.', 1000*clearance
        )
    end

    return true
end

return Validation