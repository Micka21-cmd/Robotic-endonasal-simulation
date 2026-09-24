-- Robot joint access for kinematic experiments.
-- Joint positions are expressed in radians.

local Robot = {}
Robot.__index = Robot

function Robot.new(simApi, rootHandle, jointPath, jointCount)
    local self = setmetatable({}, Robot)

    self.sim = simApi
    self.root = rootHandle
    self.joints = {}

    for i = 1, jointCount do
        self.joints[i] = self.sim.getObject(
            jointPath,
            {index = i - 1}
        )
    end

    return self
end

function Robot:readConfiguration()
    local configuration = {}

    for i, joint in ipairs(self.joints) do
        configuration[i] = self.sim.getJointPosition(joint)
    end

    return configuration
end

function Robot:setConfiguration(configuration)
    for i, joint in ipairs(self.joints) do
        self.sim.setJointPosition(joint, configuration[i])
    end
end

return Robot