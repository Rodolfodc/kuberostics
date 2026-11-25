-- Laser sweep script of the combined myRobot + P3 laser model (child script of /myRobot/LaserScanner_2D).
-- Derived from the P3 (slam_toolbox) laser script. Differences: no graph object calls (debug only, they cost time),
-- the unused point cloud is not computed, and the sweep can run every Nth step.
-- Contract kept: float array signal "distSignal" on /myRobot, default 452 beams, -90..+90.4 deg, 0 for a miss
-- (measured: handleProximitySensor returns 0 for a miss in the House scene, so distSignal had zeros and never the
-- stock script's range_max + 1 fallback; the zero is now set explicitly from the detection flag).
-- Settings (model customData on LaserScanner_2D, optional):
--   scanEveryN  (int, default 1)   sweep only every Nth simulation step; distSignal keeps its last value in between
--   (diagnostic output: float signal laserSweepSeconds = wall time of the last sweep)
--   scanDensity (float, default 2.5) beams per degree over the 180 deg sweep (2.5 -> 452 beams; changes the signal length)
sim = require 'sim'

function sysCall_init()
    robotHandle = sim.getObject('/myRobot')
    laserHandle = sim.getObjectHandle("LaserScannerLaser_2D")
    jointHandle = sim.getObjectHandle("LaserScannerJoint_2D")
    modelHandle = sim.getObjectHandle(sim.handle_self)
    stepCounter = 0
end

local function setting(name, default)
    local v = sim.getStringProperty(modelHandle, 'customData.' .. name, {noError = true})
    return tonumber(v) or default
end

function sysCall_sensing()
    local every = math.max(1, math.floor(setting('scanEveryN', 1)))
    local n = stepCounter
    stepCounter = stepCounter + 1
    if n % every ~= 0 then return end
    local t0 = sim.getSystemTime()
    local scanningAngle = 180
    local density = math.min(5, math.max(0.1, setting('scanDensity', 2.5)))
    local pts = scanningAngle * density + 1
    local p = -scanningAngle * math.pi / 360
    local stepSize = math.pi / (density * 180)
    local distance = {}
    for i = 0, pts, 1 do
        sim.setJointPosition(jointHandle, p)
        p = p + stepSize
        local r, dist = sim.handleProximitySensor(laserHandle)
        if r <= 0 or dist == nil then
            dist = 0 -- miss
        end
        distance[#distance + 1] = dist
    end
    sim.setFloatArrayProperty(robotHandle, "signal.distSignal", distance)
    sim.setFloatSignal("laserSweepSeconds", sim.getSystemTime() - t0) -- wall time of the last sweep (diagnostics)
end

function sysCall_cleanup()
end
