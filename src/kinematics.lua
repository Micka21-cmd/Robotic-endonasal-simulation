-- Inverse kinematics for prescribed tool motion.
-- Candidate configurations are computed in the IK environment.
-- Applying them to the scene remains the caller's responsibility.

local Kinematics = {}
Kinematics.__index = Kinematics

function Kinematics.new(ikApi, robot, config)
    local self = setmetatable({}, Kinematics)

    self.ik = ikApi
    self.robot = robot
    self.config = config

    self.environment = nil
    self.group = nil
    self.jointMap = nil

    return self
end

function Kinematics:initialize(tip, target)
    assert(self.environment == nil, 'IK is already initialized')

    local ik = self.ik

    self.environment = ik.createEnvironment()
    self.group = ik.createGroup(self.environment)

    ik.setGroupCalculation(
        self.environment,
        self.group,
        ik.method_damped_least_squares,
        self.config.ikDamping,
        self.config.ikMaxIterations
    )

    -- Constrain position and tool-axis direction.
    -- Rotation about the tool's local Z axis remains unconstrained.
    local element
    element, self.jointMap = ik.addElementFromScene(
        self.environment,
        self.group,
        self.robot.root,
        tip,
        target,
        ik.constraint_position + ik.constraint_alpha_beta
    )

    ik.setElementPrecision(
        self.environment,
        self.group,
        element,
        {
            self.config.ikPositionTolerance,
            self.config.ikOrientationTolerance
        }
    )

    for _, joint in ipairs(self.robot.joints) do
        ik.setJointMode(
            self.environment,
            self.jointMap[joint],
            ik.jointmode_ik
        )
    end
end

function Kinematics:syncFromScene()
    self.ik.syncFromSim(self.environment, {self.group})
end

function Kinematics:solveCandidate()
    -- Restart from the current scene configuration at each attempt.
    self:syncFromScene()

    local result, flags = self.ik.handleGroup(
        self.environment,
        self.group
    )

    if result ~= self.ik.result_success then
        return nil, flags
    end

    local candidate = {}
    for i, joint in ipairs(self.robot.joints) do
        candidate[i] = self.ik.getJointPosition(
            self.environment,
            self.jointMap[joint]
        )
    end

    return candidate, flags
end

function Kinematics:destroy()
    if self.environment ~= nil then
        self.ik.eraseEnvironment(self.environment)
        self.environment = nil
        self.group = nil
        self.jointMap = nil
    end
end

return Kinematics