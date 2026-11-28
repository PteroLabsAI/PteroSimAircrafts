# Omnicopter

The PX4 Gazebo Classic Omnicopter has eight tilted, reversible propellers
inside a protective cage. The eight motors can generate force and torque
on all three body axes. See [NOTICE](NOTICE) for the source model and meshes.

## PteroSim and PX4 SITL

Spawn the configurable aircraft named `omnicopter` and select the PX4 flight stack.
The physics rate must be at least 240 Hz. The IMU, including its magnetic field,
runs at 240 Hz, the barometer at 48 Hz and the GPS at 5 Hz. At the default
physics rate each sensor has an even sample interval.

Use the supplied [PX4 airframe](firmwares/px4_omnicopter) in a PX4 POSIX SITL
build with the MAVLink simulator interface. Install it under
`etc/init.d-posix/airframes` in the SITL root filesystem as
`19601_pterosim_omnicopter`, then launch SITL with
`PX4_SIM_MODEL=pterosim_omnicopter`. The PteroSim PX4 connector listens on
TCP 4560 for the first aircraft. Motor functions 101 through 108 map to
signed HIL channels 0 through 7.

PX4's standard multicopter controller uses the vertical thrust axis for
normal position control. Testing independent lateral force requires a
controller that commands the full body thrust vector.

## Model details

The JSBSim model combines the SDF body, rotor, IMU and GPS masses and
inertias. Contact points follow the eight feet of the cage mesh. Each
rotor applies signed quadratic thrust at its SDF mount, with first-order
motor response, axial airspeed attenuation and shaft reaction torque.
The rotor clips are driven by phase from the corresponding signed rotor
speed with the original 10:1 visual slowdown.

The model uses zero as the neutral command for all eight rotors. It omits
the Gazebo Classic armed-only offset on channel 3, rotor transverse drag,
and wind-induced rolling moments. Each rotor's axial airspeed is measured
at the aircraft origin.
