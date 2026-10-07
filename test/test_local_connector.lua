-- Tests the actual connector implementation with simulated scene/IK services.
local Test={}
function Test.run(LocalConnector)
    local function copy(t) local r={};for i,v in ipairs(t) do r[i]=v end;return r end
    local function same(a,b) if #a~=#b then return false end;for i,v in ipairs(a) do if v~=b[i] then return false end end;return true end
    local count=0
    local function check(v,m) assert(v,m);count=count+1 end
    local entry={9,8,7,0,0,0}
    local originalTarget={1,2,3,0,0,0,1}
    local current,target=copy(entry),copy(originalTarget)
    local mode='success'
    local robot={joints={1,2,3,4,5,6}}
    function robot:readConfiguration() return copy(current) end
    function robot:setConfiguration(v) current=copy(v) end
    local sim={handle_world=-1}
    function sim.getObjectPosition() return {current[1],current[2],current[3]} end
    function sim.getObjectPose(handle)
        if handle=='target' then return copy(target) end
        return {current[1],current[2],current[3],0,0,0,1}
    end
    function sim.setObjectPose(_,pose) target=copy(pose) end
    local syncCount=0
    local ik={}
    function ik:syncFromScene() syncCount=syncCount+1 end
    function ik:solveCandidate()
        if mode=='lua_error' then error('Injected IK error') end
        if mode=='ik_failure' then return nil,123 end
        return {target[1],target[2],target[3],0,0,0}
    end
    local validator={}
    function validator:checkJoints(a,b)
        if math.abs(a[6]-b[6])>0.5 then return false,'joint step' end
        return true
    end
    function validator:checkState() return true end
    local segment={}
    function segment.check(_,_,a,b)
        if mode=='blocked_correction' and a[6]~=b[6] then return false,'blocked correction' end
        return true
    end
    local trajectory={poseAt=function(_,_,p) return {p[1],p[2],p[3],0,0,0,1} end}
    local c=LocalConnector.new(sim,robot,ik,validator,{SegmentValidation=segment,Trajectory=trajectory},
        {target='target',tip='tip',anatomy='anatomy'},
        {localConnectorStepM=0.1,localConnectorMaxSteps=100,segmentResolutionRad=0.01})
    local start={0,0,1,0,0,0};local goal={0.2,0,1,0,0,0.01}
    local function restored()
        check(same(current,entry),'Robot not restored')
        check(same(target,originalTarget),'Target not restored')
    end
    local r=c:connectConfiguration(start,goal)
    check(r.success and same(r.path[#r.path],goal),'Exact fixed endpoint not reached')
    check(#r.path==4 and #r.tipPositions==#r.path and #r.targetPoses==#r.path,'Correction metadata missing')
    check(r.endpointCorrectionRad==0.01,'Correction magnitude incorrect')
    restored()
    mode='blocked_correction';r=c:connectConfiguration(start,goal)
    check(not r.success,'Invalid correction accepted');restored()
    mode='success';local large=copy(goal);large[6]=0.6
    r=c:connectConfiguration(start,large)
    check(not r.success,'Joint step bypassed');restored()
    mode='ik_failure';r=c:connectConfiguration(start,goal)
    check(not r.success,'IK failure accepted');restored()
    mode='lua_error';local ok=pcall(function() c:connectConfiguration(start,goal) end)
    check(not ok,'Lua error swallowed');restored()
    mode='success';goal[6]=0;r=c:connectConfiguration(start,goal)
    check(r.success and #r.path==3 and #r.tipPositions==3,'Zero correction added duplicate')
    restored()
    check(syncCount>=12,'IK synchronization missing')
    print(string.format('LOCAL CONNECTOR UNIT TESTS: PASS; %d assertions.',count))
    return true
end
return Test
