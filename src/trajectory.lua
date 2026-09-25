-- Prescribed straight-line tip trajectory in the anatomy frame.
-- The RCM is located at the anatomy-frame origin.
-- Positions are in metres; quaternion order is qx, qy, qz, qw.

local Trajectory = {}

function Trajectory.poseAt(progressTime, duration, startPosition, goalPosition)
    local u = progressTime / duration
    local s = 3*u*u - 2*u*u*u

    local x = startPosition[1]
        + (goalPosition[1] - startPosition[1])*s
    local y = startPosition[2]
        + (goalPosition[2] - startPosition[2])*s
    local z = startPosition[3]
        + (goalPosition[3] - startPosition[3])*s

    -- This trajectory model supports the positive-Z anatomical workspace.
    assert(z > 0, 'Prescribed tip position must have positive anatomical Z')

    local radial = math.sqrt(x*x + y*y)
    local qx, qy, qz, qw = 0, 0, 0, 1

    if radial > 0 then
        local angle = math.atan(radial / z)
        local halfSin = math.sin(angle / 2)

        qx = -(y / radial)*halfSin
        qy =  (x / radial)*halfSin
        qw = math.cos(angle / 2)
    end

    return {x, y, z, qx, qy, qz, qw}
end

return Trajectory