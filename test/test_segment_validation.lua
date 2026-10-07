-- Isolated software test: valid endpoints, invalid interior.
-- Does not access or modify the CoppeliaSim scene.

return function(SegmentValidation)
    local currentQ = {0.25}
    local visited = {}

    local robot = {
        joints = {1},
    }

    function robot:readConfiguration()
        return {currentQ[1]}
    end

    function robot:setConfiguration(q)
        currentQ = {q[1]}
    end

    local validator = {}

    function validator:checkJoints(candidate, previous)
        return true
    end

    function validator:checkState(checkTracking)
        assert(
            checkTracking == false,
            'Segment checks must disable tip tracking'
        )

        local q = currentQ[1]
        visited[#visited + 1] = q

        if q >= 0.4 and q <= 0.6 then
            return false, 'Synthetic invalid interior.'
        end

        return true
    end

    -- Establish independently that both endpoints are valid.
    robot:setConfiguration({0})
    assert(validator:checkState(false), 'Start must be valid')

    robot:setConfiguration({1})
    assert(validator:checkState(false), 'Goal must be valid')

    -- Restore the test entry configuration and clear observations.
    robot:setConfiguration({0.25})
    visited = {}

    -- Resolution 1 gives two intervals with the current module:
    -- q = 0, q = 0.5, q = 1.
    local valid, reason = SegmentValidation.check(
        robot,
        validator,
        {0},
        {1},
        1
    )

    local restoredQ = robot:readConfiguration()

    assert(valid == false, 'Invalid interior must reject the segment')

    assert(
        type(reason) == 'string'
            and reason:find('Synthetic invalid interior.', 1, true),
        'Rejection must come from the interior geometry check'
    )

    assert(
        #visited == 2
            and math.abs(visited[1]) <= 1e-12
            and math.abs(visited[2] - 0.5) <= 1e-12,
        'Expected start and midpoint checks, then early rejection'
    )

    assert(
        math.abs(restoredQ[1] - 0.25) <= 1e-12,
        'Entry configuration must be restored after rejection'
    )

    print(
        'SEGMENT INTERIOR TEST: PASS; valid endpoints; '
        .. 'midpoint rejected; entry configuration restored.'
    )
end