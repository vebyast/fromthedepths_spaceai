-- Author: vebyast@gmail.com
-- don't really care about the license. do what you want with the code

--######################################################################
--######################################################################
--### general library things
--######################################################################
--######################################################################

-- forward: (1, 0, 0), x, left-handed rotation is roll
-- up:      (0, 1, 0), y, left-handed rotation is yaw
-- right:   (0, 0, 1), z, left-handed rotation is pitch

local THRUST = {
   FORWARD = 0,
   BACKWARD = 1,
   RIGHT = 2,
   LEFT = 3,
   UP = 4,
   DOWN = 5,
   ROLL_R = 6,
   ROLL_L = 7,
   YAW_R = 8,
   YAW_L = 9,
   PITCH_UP = 10,
   PITCH_DOWN = 11,
}; 

function PIDcreate(arg)
   local pid = {};
   pid.K_p = arg.K_p;
   pid.K_d = arg.K_d;
   pid.K_i = arg.K_i;
   pid.err_last = 0;
   pid.err_integral = 0;
   pid.setpoint = arg.setpoint;
   return pid;
end

function PIDtick(arg)
   local self = arg[1]
   local err = self.setpoint - arg.cur;
   local P_control = err * self.K_p;

   local err_D = err - self.err_last;
   local D_control = err_D * self.K_d;
   self.err_last = err;

   self.err_integral = self.err_integral + err;
   local I_control = self.err_integral * self.K_i;
   
   local control = P_control + D_control + I_control;
   return control;
end

function normalizeAngle(angle)
   while (angle > 180) do angle = angle - 360; end
   while (angle < -180) do angle = angle + 360; end
   return angle;
end

function testControl(arg)
   local I = arg[1]
   for idx = 0, I:Component_GetCount(6) - 1 do
      loc = I:Component_GetLocalPosition(6, idx);
      if (arg.coords.x == loc.x and arg.coords.y == loc.y and arg.coords.z == loc.z) then
         return I:Component_GetBoolLogic(6, idx);
      end
   end
   return False;
end

function applyLocalThrust(arg)
   local I = arg[1]
   local v = arg.v
   -- I:Log("applying local thrust: " .. tostring(v))

   if (v.x > 0) then I:RequestThrustControl(THRUST.FORWARD, v.x); end
   if (v.x < 0) then I:RequestThrustControl(THRUST.BACKWARD, -v.x); end

   if (v.y > 0) then I:RequestThrustControl(THRUST.UP, v.y); end
   if (v.y < 0) then I:RequestThrustControl(THRUST.DOWN, -v.y); end

   if (v.z > 0) then I:RequestThrustControl(THRUST.RIGHT, v.z); end
   if (v.z < 0) then I:RequestThrustControl(THRUST.LEFT, -v.z); end
end

function applyGlobalThrust(arg)
   local I = arg[1]
   local v = arg.v
   -- I:Log("applying global thrust: " .. tostring(v))

   local x = Vector3.Dot(v, I:GetConstructForwardVector())
   local y = Vector3.Dot(v, I:GetConstructUpVector())
   local z = Vector3.Dot(v, I:GetConstructRightVector())
   local localThrust = Vector3(x, y, z)
   applyLocalThrust{I, v=localThrust}
end

function applyLocalRotationThrust(arg)
   local I = arg[1]
   local v = arg.v
   -- I:Log("applying local rotation thrust: " .. tostring(v))
   
   if (v.x > 0) then I:RequestThrustControl(THRUST.ROLL_L, v.x); end
   if (v.x < 0) then I:RequestThrustControl(THRUST.ROLL_R, -v.x); end

   if (v.y > 0) then I:RequestThrustControl(THRUST.YAW_R, v.y); end
   if (v.y < 0) then I:RequestThrustControl(THRUST.YAW_L, -v.y); end

   if (v.z > 0) then I:RequestThrustControl(THRUST.PITCH_DOWN, v.z); end
   if (v.z < 0) then I:RequestThrustControl(THRUST.PITCH_UP, -v.z); end
end

-- shamelessly stolen from Madwand at
-- http://www.fromthedepthsgame.com/forum/showthread.php?tid=9642 and then tweaked a bit
function EulerAngles(arg)
   local q1 = arg[1]
   local sqw = q1.w*q1.w
   local sqx = q1.x*q1.x
   local sqy = q1.y*q1.y
   local sqz = q1.z*q1.z
   local gimbal = (q1.x * q1.z) + (q1.w * q1.y)
   local rx, ry, rz
   if (gimbal > 0.499) then --singularity at north pole
      rx = 0
      ry = math.pi/2;
      rz = 2 * math.atan2(q1.x,q1.w)
   elseif (gimbal < -0.499) then --singularity at south pole
      rx = 0
      ry = -math.pi/2
      rz = -2 * math.atan2(q1.x,q1.w)
   else
      rx = math.atan2((q1.w * q1.x)+(q1.y * q1.z), .5 - (sqx + sqy))
      ry = math.asin(2*((q1.w * q1.y) - (q1.z * q1.x)))
      rz = math.atan2((q1.w * q1.z)+(q1.x * q1.y), .5 - (sqy + sqz))
   end
   return Vector3(math.deg(rx), math.deg(ry), math.deg(rz))
end


--######################################################################
--######################################################################
--### vehicle-specific things
--######################################################################
--######################################################################


-- vehicle constants

-- local weaponDirection = Vector3(0, -1, 0)

local neutralPosition = {
   y = 2000,
   rx = 0,
   ry = 0,
   rz = 0,
}

-- -- vehicle PID controllers

local pids = {
   x = { pid = PIDcreate{K_p=.01, K_d=.5, K_i=0, setpoint=neutralPosition.x or 0},
         get = function(I) return I:GetConstructPosition().x end,
         active = true},
   y = { pid = PIDcreate{K_p=.01, K_d=.5, K_i=0, setpoint=neutralPosition.y or 0},
         get = function(I) return I:GetConstructPosition().y end, 
         active = true},
   z = { pid = PIDcreate{K_p=.01, K_d=.5, K_i=0, setpoint=neutralPosition.z or 0},
         get = function(I) return I:GetConstructPosition().z end, 
         active = true},

   rx = { pid = PIDcreate{K_p=.06, K_d=3., K_i=0, setpoint=neutralPosition.rx or 0},
          get = function(I) return normalizeAngle(I:GetConstructRoll()) end, 
         active = true},
   ry = { pid = PIDcreate{K_p=.06, K_d=3., K_i=0, setpoint=neutralPosition.ry or 0},
          get = function(I) return normalizeAngle(I:GetConstructYaw()) end, 
         active = true},
   rz = { pid = PIDcreate{K_p=.06, K_d=3., K_i=0, setpoint=neutralPosition.rz or 0},
          get = function(I) return normalizeAngle(I:GetConstructPitch()) end, 
         active = true},
}

-- -- vehicle functions

function resetPIDs()
   pids.x.err_integral = 0
   pids.y.err_integral = 0
   pids.z.err_integral = 0

   pids.rx.pid.err_integral = 0
   pids.ry.pid.err_integral = 0
   pids.rz.pid.err_integral = 0
end

function tickPIDs(arg)
   local I = arg[1]
   local thrust = Vector3(0, 0, 0);
   local rotate = Vector3(0, 0, 0);

   thrust.x = PIDtick{pids.x.pid, cur=pids.x.get(I)}
   thrust.y = PIDtick{pids.y.pid, cur=pids.y.get(I)}
   thrust.z = PIDtick{pids.z.pid, cur=pids.z.get(I)}

   rotate.x = PIDtick{pids.rx.pid, cur=pids.rx.get(I)}
   rotate.y = PIDtick{pids.ry.pid, cur=pids.ry.get(I)}
   rotate.z = PIDtick{pids.rz.pid, cur=pids.rz.get(I)}

   if not pids.x.active then thrust.x = 0 end
   if not pids.y.active then thrust.y = 0 end
   if not pids.z.active then thrust.z = 0 end
   if not pids.rx.active then rotate.x = 0 end
   if not pids.ry.active then rotate.y = 0 end
   if not pids.rz.active then rotate.z = 0 end
   
   -- I:Log("    -> " .. rotate.x)
   -- I:Log("  x/roll: " .. pids.rx.get(I))
   -- I:Log("    -> " .. rotate.y)
   -- I:Log("  y/yaw: " .. pids.ry.get(I))
   -- I:Log("    -> " .. rotate.z)
   -- I:Log("  z/pitch: " .. pids.rz.get(I))

   return thrust, rotate
end

function pointAtTarget(arg)
   local I = arg[1]
   local target = I:GetTargetInfo(0, 0)
   for tidx = 0, I:GetNumberOfTargets(0) - 1 do
      local t = I:GetTargetInfo(0, tidx)
      if t.Valid and t.PlayerTargetChoice then
         target = t
      end
   end

   if target.Valid then
      local toTarget = target.AimPointPosition - I:GetConstructPosition()
      toTarget.x, toTarget.z = -toTarget.z, -toTarget.x
      local weaponTargetRotation = Quaternion.FromToRotation(weaponDirection, toTarget)
      local weaponTargetRotationRPY = EulerAngles{weaponTargetRotation}
      pids.rx.pid.setpoint = weaponTargetRotationRPY.x
      pids.ry.pid.setpoint = weaponTargetRotationRPY.y
      pids.rz.pid.setpoint = weaponTargetRotationRPY.z
      
      pids.rx.active = true
      pids.ry.active = true
      pids.rz.active = true

      -- I:Log("target valid, pointing at " .. tostring(toTarget.normalized))
      -- I:Log("  weapon direction rotated correctly: " .. tostring(weaponTargetRotation * weaponDirection))
      -- I:Log("  weapon direction rotated by eulers: " .. tostring(Quaternion.Euler(weaponTargetRotationRPY) * weaponDirection))
   end
end

local rememberedPosition = {}
function doStationkeeping(arg)
   local I = arg[1]
   local pos = I:GetConstructPosition()

   -- turning on the rightmost fuel processor tells the ship to assume its neutral y position
   if (testControl{I, coords=Vector3(4, 3, -4)}) then
      I:LogToHud("assuming neutral altitude")
      pids.y.pid.setpoint = neutralPosition.y
      pids.y.active = true
   end

   -- turning on the middle fuel processor tells the ship to hold xz position
   if (testControl{I, coords=Vector3(3, 3, -4)}) then
      I:LogToHud("holding position")
      if rememberedPosition.x == nil then rememberedPosition.x = pos.x end
      if rememberedPosition.z == nil then rememberedPosition.z = pos.z end
      pids.x.pid.setpoint = rememberedPosition.x
      pids.z.pid.setpoint = rememberedPosition.z
      pids.x.active = true
      pids.z.active = true
   else
      rememberedPosition.x = nil
      rememberedPosition.z = nil
   end

   -- turning on the left processor tells the ship to hold altitude
   if (testControl{I, coords=Vector3(2, 3, -4)}) then
      I:LogToHud("holding altitude")
      if rememberedPosition.y == nil then rememberedPosition.y = pos.y end
      pids.y.pid.setpoint = rememberedPosition.y
      pids.y.active = true
   else
      rememberedPosition.y = nil
   end
end

-- main

function Update(I)
   I:ClearLogs()

   pids.x.active = false
   pids.y.active = false
   pids.z.active = false
   pids.rx.active = false
   pids.ry.active = false
   pids.rz.active = false

   -- tests several fuel processors and sets translational PID setpoints in various ways. there's a
   -- "hold current altitude" setting, a "hold neutral altitude" setting, and a "hold current
   -- horizontal position" setting.
   doStationkeeping{I}

   -- set the setpoints of the rotation PIDs so we either point in our neutral orientation or point
   -- our weapon direction at our current target
   pointAtTarget{I}

   -- now that we've set our setpoints, tick all the PIDs
   local thrust, rotate = tickPIDs{I}

   -- if the leftmost fuel processor is active, pass our calculated controls on to the engines
   if (testControl{I, coords=Vector3(-2, 3, -4)}) then
      applyGlobalThrust{I, v=thrust}
      applyLocalRotationThrust{I, v=rotate}
   else
   end
end
