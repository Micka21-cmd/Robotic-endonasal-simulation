-- Workspace-sampled RRT* variant with fixed joint configurations at nodes.
-- Cost: polyline length of measured tip positions at stored configurations.
-- Sampled IK/RCM validation does not establish asymptotic optimality.
local RRTStar = {}

local function copy(t)
    local r = {}
    for i, v in ipairs(t) do r[i] = v end
    return r
end

local function finite(x)
    return type(x) == 'number' and x == x and math.abs(x) < math.huge
end

local function distance(a, b)
    local sum = 0
    for i = 1, 3 do sum = sum + (a[i] - b[i])^2 end
    return math.sqrt(sum)
end

local function vector(t, size, label)
    assert(type(t) == 'table' and #t == size, label .. ': invalid size')
    for i = 1, size do assert(finite(t[i]), label .. ': non-finite value') end
end

local function sameQ(a, b)
    if #a ~= #b then return false end
    for i = 1, #a do if a[i] ~= b[i] then return false end end
    return true
end

function RRTStar.plan(startQ, startPose, goal, connector, config, options)
    options = options or {}
    local lo, hi = config.rrtBoundsMin, config.rrtBoundsMax
    local step, budget = config.rrtExtensionM, config.rrtMaxIterations
    local bias, seed = config.rrtGoalBias, config.rrtSeed
    local radiusMax, gamma = config.rrtStarRadiusMaxM, config.rrtStarGammaM
    local epsilon = 1e-12 -- Cost improvement tolerance, metres.
    local positionTolerance = 1e-9 -- Connector metadata consistency only.
    local nq = #startQ

    assert(nq > 0, 'Empty start configuration')
    vector(startQ, nq, 'Start configuration')
    vector(startPose, 7, 'Start pose')
    vector(goal, 3, 'Goal')
    vector(lo, 3, 'Lower bounds')
    vector(hi, 3, 'Upper bounds')
    assert(finite(step) and step > 0, 'Invalid extension')
    assert(finite(budget) and budget >= 1 and budget == math.floor(budget),
        'Invalid iteration budget')
    assert(finite(bias) and bias >= 0 and bias <= 1, 'Invalid goal bias')
    assert(finite(seed) and seed >= 1 and seed < 2147483647
        and seed == math.floor(seed), 'Invalid seed')
    assert(finite(radiusMax) and radiusMax > 0, 'Invalid neighbour radius')
    assert(finite(gamma) and gamma > 0, 'Invalid radius coefficient')
    assert(finite(config.maxTipError) and config.maxTipError >= 0,
        'Invalid goal tolerance')
    assert(lo[3] > 0, 'Sampling bounds require positive anatomical Z')
    assert(type(connector.connectConfiguration) == 'function',
        'Fixed-configuration connector is required')
    for i = 1, 3 do
        assert(lo[i] < hi[i], 'Invalid sampling bounds')
        assert(startPose[i] >= lo[i] and startPose[i] <= hi[i]
            and goal[i] >= lo[i] and goal[i] <= hi[i],
            'Start and goal must lie inside sampling bounds')
    end

    local randomState = seed
    local function random()
        randomState = (16807 * randomState) % 2147483647
        return randomState / 2147483647
    end
    local function sampleAt(iteration)
        local p
        if options.sample then
            -- Deterministic dependency injection for tests only.
            p = copy(options.sample(iteration))
        elseif random() < bias then
            p = copy(goal)
        else
            p = {}
            for i = 1, 3 do p[i] = lo[i] + random() * (hi[i] - lo[i]) end
        end
        vector(p, 3, 'Sample')
        for i = 1, 3 do
            assert(p[i] >= lo[i] and p[i] <= hi[i], 'Sample outside bounds')
        end
        return p
    end

    local stats = {
        seed = seed, iterations = 0, nodes = 1,
        connector_calls = 0, rejected_connections = 0,
        fixed_connector_calls = 0, parent_changes = 0, rewires = 0,
        descendant_cost_updates = 0, goal_connections = 0,
        goal_candidates = 0, solution_improvements = 0,
        out_of_bounds_extensions = 0,
        cost_definition = 'stored_actual_tip_polyline_length_m',
        stop_reason = 'iteration_budget',
    }
    local nodes = {{q = copy(startQ), p = {startPose[1], startPose[2], startPose[3]},
        cost = 0, children = {}}}
    -- Goal connections are terminal candidates, not expansion nodes.
    local goals, history = {}, {}
    local bestGoal, bestCost = nil, math.huge

    local function checkedEdge(source, destinationQ, destinationP, fixed)
        stats.connector_calls = stats.connector_calls + 1
        local edge
        if fixed then
            stats.fixed_connector_calls = stats.fixed_connector_calls + 1
            edge = connector:connectConfiguration(source.q, destinationQ)
        else
            edge = connector:connect(source.q, destinationP)
        end
        assert(type(edge.success) == 'boolean', 'Missing connector status')
        if not edge.success then
            stats.rejected_connections = stats.rejected_connections + 1
            return nil
        end
        assert(type(edge.path) == 'table' and #edge.path >= 1,
            'Missing connector path')
        assert(type(edge.tipPositions) == 'table'
            and #edge.tipPositions == #edge.path, 'Missing measured tip positions')
        assert(type(edge.targetPoses) == 'table'
            and #edge.targetPoses == #edge.path, 'Missing reference poses')
        for i = 1, #edge.path do
            vector(edge.path[i], nq, 'Path configuration')
            vector(edge.tipPositions[i], 3, 'Measured tip position')
            vector(edge.targetPoses[i], 7, 'Reference pose')
        end
        vector(edge.reachedPosition, 3, 'Reached position')
        assert(sameQ(edge.path[1], source.q), 'Connector changed its entry configuration')
        assert(distance(edge.tipPositions[1], source.p) <= positionTolerance,
            'Connector entry position is inconsistent')
        assert(distance(edge.tipPositions[#edge.path], edge.reachedPosition)
            <= positionTolerance, 'Connector endpoint position is inconsistent')
        if fixed then
            assert(sameQ(edge.path[#edge.path], destinationQ),
                'Connector did not reach the exact destination configuration')
            assert(distance(edge.reachedPosition, destinationP) <= positionTolerance,
                'Fixed-node position is inconsistent')
        end
        local cost = 0
        for i = 2, #edge.tipPositions do
            cost = cost + distance(edge.tipPositions[i - 1], edge.tipPositions[i])
        end
        assert(finite(cost), 'Non-finite edge cost')
        edge.cost = cost
        return edge
    end

    local function isAncestor(ancestor, index)
        while index do
            if index == ancestor then return true end
            index = nodes[index].parent
        end
        return false
    end
    local function reparent(index, parent, edge)
        assert(index ~= 1 and not isAncestor(index, parent), 'Tree cycle rejected')
        local node = nodes[index]
        local children = nodes[node.parent].children
        for j, child in ipairs(children) do
            if child == index then table.remove(children, j); break end
        end
        node.parent, node.edge = parent, edge
        node.cost = nodes[parent].cost + edge.cost
        nodes[parent].children[#nodes[parent].children + 1] = index
        local queue = {index}
        local head = 1
        while head <= #queue do
            local current = nodes[queue[head]]
            head = head + 1
            for _, child in ipairs(current.children) do
                nodes[child].cost = current.cost + nodes[child].edge.cost
                stats.descendant_cost_updates = stats.descendant_cost_updates + 1
                queue[#queue + 1] = child
            end
        end
    end
    local function refreshBest()
        local selected, cost = nil, math.huge
        for i, terminal in ipairs(goals) do
            local c = nodes[terminal.parent].cost + (terminal.edge and terminal.edge.cost or 0)
            if c < cost then selected, cost = i, c end
        end
        if not selected then return end
        bestGoal = selected
        if cost < bestCost - epsilon then
            bestCost = cost
            stats.best_cost_m = cost
            if stats.first_solution_iteration == nil then
                stats.first_solution_iteration = stats.iterations
                stats.first_solution_cost_m = cost
                stats.first_solution_connector_calls = stats.connector_calls
            else
                stats.solution_improvements = stats.solution_improvements + 1
            end
            history[#history + 1] = {
                stats.iterations, stats.connector_calls, cost
            }
            if options.log then
                options.log(string.format(
                    'RRT* solution: iteration=%d; cost=%.6f mm; calls=%d.',
                    stats.iterations, 1000 * cost, stats.connector_calls))
            end
        end
        -- Keep the exact selected cost even for sub-epsilon differences.
        bestCost = cost
        stats.best_cost_m = cost
    end

    if distance(nodes[1].p, goal) <= 1e-12 then
        goals[1] = {parent = 1}
        stats.goal_candidates = 1
        refreshBest()
    end

    for iteration = 1, budget do
        stats.iterations = iteration
        local sample = sampleAt(iteration)
        local nearest, nearestDistance = 1, math.huge
        for i, node in ipairs(nodes) do
            local d = distance(node.p, sample)
            if d < nearestDistance then nearest, nearestDistance = i, d end
        end
        if nearestDistance > 1e-12 then
            local desired = {}
            local fraction = math.min(1, step / nearestDistance)
            for i = 1, 3 do
                desired[i] = nodes[nearest].p[i]
                    + fraction * (sample[i] - nodes[nearest].p[i])
            end
            local edge = checkedEdge(nodes[nearest], nil, desired, false)
            local inside = edge ~= nil
            if edge then
                for i = 1, 3 do
                    if edge.reachedPosition[i] < lo[i] or edge.reachedPosition[i] > hi[i] then
                        inside = false
                    end
                end
                if not inside then
                    stats.out_of_bounds_extensions = stats.out_of_bounds_extensions + 1
                end
            end
            if inside then
                local q = copy(edge.path[#edge.path])
                local p = copy(edge.reachedPosition)
                local n = #nodes + 1
                -- Workspace neighbourhood schedule; no theoretical guarantee claimed.
                local radius = math.min(radiusMax, gamma * (math.log(n) / n)^(1 / 3))
                stats.last_radius_m = radius
                local near = {}
                for i, node in ipairs(nodes) do
                    if distance(node.p, p) <= radius then near[#near + 1] = i end
                end
                table.sort(near, function(a, b)
                    local ca = nodes[a].cost + distance(nodes[a].p, p)
                    local cb = nodes[b].cost + distance(nodes[b].p, p)
                    if ca == cb then return a < b end
                    return ca < cb
                end)
                local parent, cost = nearest, nodes[nearest].cost + edge.cost
                for _, i in ipairs(near) do
                    local lowerBound = nodes[i].cost + distance(nodes[i].p, p)
                    if lowerBound >= cost - epsilon then break end
                    if i ~= nearest then
                        local alternative = checkedEdge(nodes[i], q, p, true)
                        if alternative and nodes[i].cost + alternative.cost < cost - epsilon then
                            parent, edge = i, alternative
                            cost = nodes[i].cost + alternative.cost
                        end
                    end
                end
                if parent ~= nearest then stats.parent_changes = stats.parent_changes + 1 end
                local index = #nodes + 1
                nodes[index] = {q = q, p = p, parent = parent,
                    edge = edge, cost = cost, children = {}}
                nodes[parent].children[#nodes[parent].children + 1] = index
                stats.nodes = #nodes

                -- Restore deterministic index order for rewiring.
                table.sort(near)
                for _, i in ipairs(near) do
                    if i ~= 1 and not isAncestor(i, index)
                        and nodes[index].cost + distance(p, nodes[i].p) < nodes[i].cost - epsilon then
                        local alternative = checkedEdge(nodes[index], nodes[i].q, nodes[i].p, true)
                        if alternative and nodes[index].cost + alternative.cost < nodes[i].cost - epsilon then
                            reparent(i, index, alternative)
                            stats.rewires = stats.rewires + 1
                        end
                    end
                end
                if distance(p, goal) <= step then
                    stats.goal_connections = stats.goal_connections + 1
                    local finalEdge = checkedEdge(nodes[index], nil, goal, false)
                    if finalEdge and distance(finalEdge.reachedPosition, goal) <= config.maxTipError then
                        goals[#goals + 1] = {parent = index, edge = finalEdge}
                        stats.goal_candidates = #goals
                    end
                end
            end
        end
        refreshBest()
        if options.log and (iteration % 5 == 0 or iteration == budget) then
            options.log(string.format(
                'RRT* progress: %d/%d; nodes=%d; calls=%d; rewires=%d; best=%s mm.',
                iteration, budget, #nodes, stats.connector_calls, stats.rewires,
                bestGoal and string.format('%.6f', 1000 * bestCost) or 'none'))
        end
    end

    local result = {success = bestGoal ~= nil, stats = stats,
        costHistory = history, method = 'rrt_star_task_space_ik'}
    if not bestGoal then
        result.reason = 'RRT* iteration budget exhausted without a goal connection.'
        return result
    end
    local terminal = goals[bestGoal]
    local chain, index = {}, terminal.parent
    while nodes[index].parent do
        chain[#chain + 1] = nodes[index].edge
        index = nodes[index].parent
    end
    local path, poses, tips = {copy(startQ)}, {copy(startPose)}, {copy(nodes[1].p)}
    local function append(edge)
        if not edge then return end
        assert(sameQ(path[#path], edge.path[1]), 'Discontinuous reconstructed joint path')
        for i = 2, #edge.path do
            path[#path + 1] = copy(edge.path[i])
            poses[#poses + 1] = copy(edge.targetPoses[i])
            tips[#tips + 1] = copy(edge.tipPositions[i])
        end
    end
    for i = #chain, 1, -1 do append(chain[i]) end
    append(terminal.edge)
    local reconstructedCost = 0
    for i = 2, #tips do reconstructedCost = reconstructedCost + distance(tips[i - 1], tips[i]) end
    assert(math.abs(reconstructedCost - bestCost) <= 1e-8,
        'Reconstructed path cost does not match propagated tree cost')
    result.path, result.targetPoses, result.tipPositions = path, poses, tips
    result.reachedPosition = copy(tips[#tips])
    result.reason = 'RRT* budget completed; best sampled valid goal path returned.'
    return result
end

return RRTStar
