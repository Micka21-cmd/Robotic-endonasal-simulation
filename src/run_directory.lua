-- Create a separate output directory for each run.
-- This implementation targets Windows.

local RunDirectory = {}

function RunDirectory.create(exportDir, runLabel, startedAt)
    assert(
        package.config:sub(1, 1) == '\\',
        'Run directory creation currently supports Windows only'
    )

    assert(
        runLabel:match('^[%w_-]+$'),
        'Run label must contain only letters, digits, underscores or hyphens'
    )

    local baseName =
        os.date('%Y%m%d_%H%M%S', startedAt) .. '_' .. runLabel

    for suffix = 0, 999 do
        local name = baseName
        if suffix > 0 then
            name = name .. string.format('_%03d', suffix)
        end

        local path = exportDir .. '/' .. name
        local windowsPath = path:gsub('/', '\\')

        -- Restrict characters before passing a path to the shell.
        assert(
            not windowsPath:find('[^%w%s_%.:\\%-]')
                and not windowsPath:find('%c'),
            'Unsupported character in export path'
        )

        -- Exit code 2 means that this name is already occupied.
        local command = 'if exist "' .. windowsPath
            .. '" (exit /b 2) else (mkdir "' .. windowsPath .. '")'

        local ok, _, code = os.execute(command)

        if ok == true or ok == 0 then
            return path, name
        end

        local exitCode = type(ok) == 'number' and ok or code
        if exitCode ~= 2 then
            error('Cannot create run directory: ' .. path)
        end
    end

    error('Cannot allocate a unique run directory')
end

return RunDirectory