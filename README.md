# fromthedepths_spaceai
Lua AI for controlling orbital weapons platforms in the game From the Depths

Control's the ship's orientation to be either in some user-defined neutral configuration or pointing 

To use:
* Set `neutralPosition` to be "neutral" configuration for your orbital platform to stay in when it's not doing anything else. `neutralPosition.x` and `.z` are unused. `neutralPosition.y` is used when you tell the stationkeeping system to hold a neutral altitude. `neutralPosition.rx`, `.ry`, and `.rz` are used when the system doesn't have a target.
* Set the various PID constants in the `pids` table to suit your ship. For each simple degree of freedom (x, y, z, rotation about x, etc) the call to `PIDcreate`  needs to be initialized with proportional, derivative, and integral constants. Proportional constants make the ship move to align with the setpoint, derivative constants make it not overshoot, and you don't care about integral constants unless you know what they are.
* Set `weaponDirection` to a `Vector3` indicating what part of your ship you want the AI to point at targets. For example, the ship I built this AI for has a great big gun pointed straight down through the middle of the ship, so I have this set to be a vector pointing straight down.
* In the `doStationkeeping` and `Update` functions, you'll need to point the magic numbers in the calls to `testControl` so they properly align with oil processors somewhere on your ship. These are used for controlling the AI without having to edit the code all the time.
