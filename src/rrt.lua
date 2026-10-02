-- Task-space RRT: nodes retain the joint configuration reached by IK.
-- Scene changes and sampled geometric checks belong to the connector.

local RRT = {}

local function copy(t)
    local r = {}
    for i, v in ipairs(t) do
        r[i] = v
    end
    return r
end

local function distance(a, b)
    local s = 0
    for i = 1, 3 do
        s = s + (a[i] - b[i])^2
    end
    return math.sqrt(s)
end

function RRT.plan(startQ, startPose, goal, connector, config)
    local lo = config.rrtBoundsMin
    local hi = config.rrtBoundsMax
    local step = config.rrtExtensionM
    local budget = config.rrtMaxIterations
    local bias = config.rrtGoalBias
    local seed = config.rrtSeed

    assert(step > 0 and step < math.huge,
        'Invalid RRT extension')
    assert(budget >= 1 and budget == math.floor(budget),
        'Invalid RRT budget')
    assert(bias >= 0 and bias <= 1,
        'Invalid RRT goal bias')
    assert(
        seed >= 1 and seed < 2147483647
            and seed == math.floor(seed),
        'Invalid RRT seed'
    )
    assert(lo[3] > 0,
        'RRT bounds must have positive anatomical Z')

    for i = 1, 3 do
        assert(
            lo[i] < hi[i]
                and lo[i] > -math.huge
                and hi[i] < math.huge,
            'Invalid RRT bounds'
        )
        assert(
            startPose[i] >= lo[i] and startPose[i] <= hi[i]
                and goal[i] >= lo[i] and goal[i] <= hi[i],
            'Start and goal must lie inside RRT bounds'
        )
    end

    -- Private generator, independent of global math.random.
    local randomState = seed

    local function random()
        randomState = (16807 * randomState) % 2147483647
        return randomState / 2147483647
    end

    local stats = {
        iterations = 0,
        nodes = 1,
        connector_calls = 0,
        rejected_connections = 0,
        seed = seed,
    }

    local nodes = {
        {
            q = copy(startQ),
            p = {startPose[1], startPose[2], startPose[3]},
        }
    }

    local function connect(q, p)
        stats.connector_calls = stats.connector_calls + 1

        local result = connector:connect(q, p)

        if not result.success then
            stats.rejected_connections =
                stats.rejected_connections + 1
        end

        return result
    end

    local function add(parent, edge)
        assert(
            edge.reachedPosition and edge.targetPoses
                and #edge.path == #edge.targetPoses,
            'Incomplete connector result'
        )

        nodes[#nodes + 1] = {
            q = copy(edge.path[#edge.path]),
            p = copy(edge.reachedPosition),
            parent = parent,
            edge = edge,
        }

        stats.nodes = #nodes
        return #nodes
    end

    local function finish(index)
        local chain = {}

        while nodes[index].parent do
            chain[#chain + 1] = index
            index = nodes[index].parent
        end

        local path = {copy(startQ)}
        local poses = {copy(startPose)}

        -- Traverse the selected branches from root to goal.
        -- Skip each edge's first sample: already in the path.
        for k = #chain, 1, -1 do
            local edge = nodes[chain[k]].edge

            for j = 2, #edge.path do
                path[#path + 1] = copy(edge.path[j])
                poses[#poses + 1] = copy(edge.targetPoses[j])
            end
        end

        return {
            success = true,
            path = path,
            targetPoses = poses,
            stats = stats,
            method = 'rrt_task_space_ik',
            reason = 'RRT reached goal through sampled valid connections.',
        }
    end

    for iteration = 1, budget do
        stats.iterations = iteration

        local sample = {}

        if random() < bias then
            sample = copy(goal)
        else
            for i = 1, 3 do
                sample[i] = lo[i] + random() * (hi[i] - lo[i])
            end
        end

        local nearest = 1
        local best = math.huge

        for i, node in ipairs(nodes) do
            local d = distance(node.p, sample)

            if d < best then
                nearest = i
                best = d
            end
        end

        if best > 1e-12 then
            local parent = nodes[nearest]
            local desired = {}
            local fraction = math.min(1, step / best)

            for i = 1, 3 do
                desired[i] = parent.p[i]
                    + fraction * (sample[i] - parent.p[i])
            end

            local edge = connect(parent.q, desired)

            if edge.success then
                local index = add(nearest, edge)
                local node = nodes[index]

                if distance(node.p, goal) <= step then
                    -- Command the exact goal before declaring success.
                    local finalEdge = connect(node.q, goal)

                    if finalEdge.success then
                        assert(
                            finalEdge.reachedPosition,
                            'Missing final reached position'
                        )

                        if distance(finalEdge.reachedPosition, goal)
                            <= config.maxTipError then
                            return finish(add(index, finalEdge))
                        end
                    end
                end
            end
        end
    end

    return {
        success = false,
        stats = stats,
        method = 'rrt_task_space_ik',
        reason = 'RRT iteration budget exhausted.',
    }
end

return RRT