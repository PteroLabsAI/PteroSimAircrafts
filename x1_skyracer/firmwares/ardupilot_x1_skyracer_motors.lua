-- X1 SkyRacer lift fans: ArduPilot's OctaQuad X (same motors, corners and spins) with
-- front/rear throttle factors. The CG sits behind the lift centre, so a hover needs 1.65 x the
-- thrust on each rear duct; putting that split into collective keeps the takeoff level.

-- Only a QuadPlane set up for a scripted matrix; any other vehicle sharing the scripts folder is left alone.
local FRAME_CLASS_SCRIPTING = 15
if param:get('Q_FRAME_CLASS') ~= FRAME_CLASS_SCRIPTING then
  return
end

-- Hover duty of a front and a rear lift fan in the flight model at race weight.
local HOVER_DUTY_FRONT = 0.6115
local HOVER_DUTY_REAR = 0.7835

-- ArduPilot's thrust units for an output duty: the inverse of its thrust curve.
local function thrust_units(duty)
  local spin_min = assert(param:get('Q_M_SPIN_MIN'), 'X1 motors: Q_M_SPIN_MIN missing')
  local spin_max = assert(param:get('Q_M_SPIN_MAX'), 'X1 motors: Q_M_SPIN_MAX missing')
  local expo = assert(param:get('Q_M_THST_EXPO'), 'X1 motors: Q_M_THST_EXPO missing')
  local r = (duty - spin_min) / (spin_max - spin_min)
  return (1 - expo) * r + expo * r * r
end

local FRONT = thrust_units(HOVER_DUTY_FRONT) / thrust_units(HOVER_DUTY_REAR)
local REAR = 1.0
local CCW, CW = 1, -1

-- {motor, roll, pitch, yaw, test order, throttle}, as AP_MotorsMatrix's OctaQuad X table;
-- motor n drives the output with SERVO function 33 + n.
local fans = {
  {0, -1,  1, CCW, 1, FRONT}, -- front right
  {1,  1,  1, CW,  7, FRONT}, -- front left
  {2,  1, -1, CCW, 5, REAR},  -- rear left
  {3, -1, -1, CW,  3, REAR},  -- rear right
  {4,  1,  1, CCW, 8, FRONT}, -- front left
  {5, -1,  1, CW,  2, FRONT}, -- front right
  {6, -1, -1, CCW, 4, REAR},  -- rear right
  {7,  1, -1, CW,  6, REAR},  -- rear left
}

for _, f in ipairs(fans) do
  MotorsMatrix:add_motor_raw(f[1], f[2], f[3], f[4], f[5])
  assert(MotorsMatrix:set_throttle_factor(f[1], f[6]), 'X1 motors: needs Q_FRAME_CLASS 15')
end
assert(MotorsMatrix:init(#fans), 'X1 motors: matrix init failed')
motors:set_frame_string('X1 SkyRacer OctaQuad X, CG trim')
