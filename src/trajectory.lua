-- Prescribed T0-to-T1 trajectory in the anatomy frame.
-- Positions are expressed in metres.
-- Quaternion convention: qx, qy, qz, qw.

local Trajectory = {}

function Trajectory.poseAt(progressTime, duration)
    local u = progressTime / duration
    local s = 3*u*u - 2*u*u*u

    local x = 0.003*s
    local z = 0.075

    -- Rotate about Y so that the tool's Z axis points from
    -- the anatomy origin (RCM) towards the prescribed tip position.
    local angle = math.atan(x / z)

    return {
        x, 0, z,
        0, math.sin(angle/2), 0, math.cos(angle/2)
    }
end

return Trajectory