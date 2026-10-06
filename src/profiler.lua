-- Temporary function instrumentation.
-- Restores the original functions after completion or Lua error.

local Profiler = {}

function Profiler.run(clock, targets, work)
    local saved, rows = {}, {}
    local active = nil

    -- Validate every target before installing any wrapper.
    for i, target in ipairs(targets) do
        local original = target.owner[target.key]

        assert(
            type(original) == 'function',
            'Cannot profile: ' .. target.name
        )

        saved[i] = {
            owner = target.owner,
            key = target.key,
            raw = rawget(target.owner, target.key),
            original = original,
        }

        rows[i] = {
            name = target.name,
            calls = 0,
            errors = 0,
            inclusive = 0,
            exclusive = 0,
        }
    end

    for i, entry in ipairs(saved) do
        local original = entry.original
        local row = rows[i]

        entry.owner[entry.key] = function(...)
            local parent = active
            local frame = {children = 0}
            active = frame

            local started = clock()
            local result = table.pack(pcall(original, ...))
            local duration = clock() - started

            active = parent

            row.calls = row.calls + 1
            row.inclusive = row.inclusive + duration
            row.exclusive =
                row.exclusive + duration - frame.children

            if parent then
                parent.children = parent.children + duration
            end

            if not result[1] then
                row.errors = row.errors + 1
                error(result[2], 0)
            end

            return table.unpack(result, 2, result.n)
        end
    end

    local result = table.pack(pcall(work))

    -- Restore table-owned functions and inherited methods.
    for i = #saved, 1, -1 do
        local entry = saved[i]
        entry.owner[entry.key] = entry.raw
    end

    if not result[1] then
        error(result[2], 0)
    end

    return rows, table.unpack(result, 2, result.n)
end

return Profiler