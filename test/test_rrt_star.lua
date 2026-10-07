-- Standalone tests; no CoppeliaSim scene required.
local Test = {}
function Test.run(RRTStar)
    local count = 0
    local function check(v, message)
        assert(v, message); count = count + 1
    end
    local function copy(t) local r = {}; for i,v in ipairs(t) do r[i]=v end; return r end
    local function q(p) return {p[1],p[2],p[3],0,0,0} end
    local function pose(p) return {p[1],p[2],p[3],0,0,0,1} end
    local function p(a) return {a[1],a[2],a[3]} end
    local function same(a,b) for i=1,#a do if a[i]~=b[i] then return false end end; return true end
    local samples = {{1,0,1},{2,0,1},{3,0,1},{0,1,1},{1,1,1}}
    local function config()
        return {rrtBoundsMin={-1,-1,0.1},rrtBoundsMax={5,5,5},
            rrtExtensionM=1.1,rrtMaxIterations=5,rrtGoalBias=0.15,rrtSeed=42,
            rrtStarRadiusMaxM=1.1,rrtStarGammaM=100,maxTipError=1e-8}
    end
    local function connector(mode)
        local c={}
        local function edge(a,b)
            if mode=='reject_all' then return {success=false,reason='blocked'} end
            local path={copy(a)}
            if same(p(a),{0,0,1}) and same(p(b),{1,0,1}) then
                path[#path+1]=q({0.5,math.sqrt(6),1})
            end
            path[#path+1]=copy(b)
            local tips,poses={},{}
            for i,v in ipairs(path) do tips[i]=p(v);poses[i]=pose(v) end
            return {success=true,path=path,tipPositions=tips,targetPoses=poses,reachedPosition=p(b)}
        end
        function c:connect(a,b)
            local e=edge(a,q(b))
            if mode=='missing_positions' then e.tipPositions=nil end
            return e
        end
        function c:connectConfiguration(a,b)
            if mode=='block_rewire' and same(p(a),{1,1,1}) and same(p(b),{1,0,1}) then
                return {success=false,reason='blocked rewire'}
            end
            local e=edge(a,b)
            if mode=='wrong_endpoint' then e.path[#e.path][6]=0.01 end
            return e
        end
        return c
    end
    local function run(mode,cfg,options)
        return RRTStar.plan(q({0,0,1}),pose({0,0,1}),{3.5,0,1},
            connector(mode),cfg or config(),options or {sample=function(i) return samples[i] end})
    end
    local r=run()
    check(r.success,'Expected success')
    check(r.stats.iterations==5,'Must continue after first solution')
    check(r.stats.first_solution_iteration==3,'Wrong first solution iteration')
    check(math.abs(r.stats.first_solution_cost_m-7.5)<1e-10,'Wrong initial cost')
    check(r.stats.parent_changes==1,'Parent selection was not exercised')
    check(r.stats.rewires==1,'Rewiring was not exercised')
    check(r.stats.descendant_cost_updates==2,'Descendant propagation incorrect')
    check(math.abs(r.stats.best_cost_m-5.5)<1e-10,'Terminal cost not updated')
    check(#r.costHistory==2 and r.costHistory[2][3]<r.costHistory[1][3],'History incorrect')
    local expected={{0,0,1},{0,1,1},{1,1,1},{1,0,1},{2,0,1},{3,0,1},{3.5,0,1}}
    check(#r.path==#expected,'Wrong reconstructed path length')
    for i,v in ipairs(expected) do check(same(r.path[i],q(v)),'Discontinuous or incorrect reconstructed path') end
    local blocked=run('block_rewire')
    check(blocked.success and blocked.stats.rewires==0,'Rejected rewire was applied')
    check(blocked.stats.rejected_connections==1,'Rejected connection not counted')
    check(math.abs(blocked.stats.best_cost_m-7.5)<1e-10,'Rejected rewire modified cost')
    local ok=pcall(function() run('wrong_endpoint') end)
    check(not ok,'Inexact fixed endpoint must fail closed')
    ok=pcall(function() run('missing_positions') end)
    check(not ok,'Missing actual positions must fail closed')
    local cfg=config();cfg.rrtMaxIterations=2
    local failed=run(nil,cfg)
    check(not failed.success and failed.path==nil,'Budget failure returned a path')
    check(failed.stats.first_solution_iteration==nil,'Invented first solution')
    failed=run('reject_all')
    check(not failed.success and failed.stats.iterations==5 and failed.stats.nodes==1,
        'Rejected extensions changed the tree')
    cfg=config();cfg.rrtMaxIterations=30
    local a,b=run(nil,cfg,{}),run(nil,cfg,{})
    check(a.success and b.success,'Seeded runs did not reach goal')
    check(a.stats.connector_calls==b.stats.connector_calls and #a.path==#b.path,'Seed not reproducible')
    for i,v in ipairs(a.path) do check(same(v,b.path[i]),'Seeded paths differ') end
    print(string.format('RRT STAR UNIT TESTS: PASS; %d assertions.',count))
    return true
end
return Test
