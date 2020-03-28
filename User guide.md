# Mars Lander User Guide

## Scenarios
Pressing any of the following numbers starts the given scenario:

0. **Circular Orbit** - Default scenario, lander is placed in a circular orbit outside of the exosphere. Autopilot is in `ORBITAL_TRANSFER` mode.

1. **Descent from 10km** - Orbiter starts with 0 absolute velocity. Autopilot is in `ORBITAL_DESCENT` mode.

2. **Elliptical orbit** - Thrust changes orbital plane.

3. **Polar launch at escape velocity** - Launch from the poles at escape velocity, but drag prevents escape from the atmosphere. Autopilot is in `ORBITAL_INJECTION` mode.

4. **Elliptical orbit that clips the atmosphere** - Orbit that clips the atmosphere, the drag from which slows the orbit and causes it to descend. Autopilot is in `ORBITAL_DESCENT` mode.

5. **Descent from 200km** - Descent from outside the atmosphere starting with 0 absolute velocity. Autopilot is in `ORBITAL_DESCENT` mode.

6. **Geostationary orbit** - Lander is placed in a Geostationary orbit above the martian surface. Autopilot is in `ORBITAL_TRANSFER` mode.

7. **Descent at perfect rotation from 10km** - as scenario 1 but lander starts at 0 velocity relative to the martian surface.

8. **Descent at perfect rotation from 200km** - as scenario 5 but lander starts at 0 velocity relative to the martian surface.

9. **Hover to 500m starting at 600m** - the lander begins at an altitude of 600m and tries to stabilise and hover to 500m.

## Autopilot Modes

In all modes, the console will post updates on any changes and what it is doing.

### `ORBITAL_RE_ENTRY`/`ORBITAL_TRANSFER`
The lander will transfer from the current orbit to a different one. Upon activating the autopilot, the simulation will freeze awaiting an input radius as a multiple of Mars’, into the console window. Upon arriving at the desired radius, the fuel will refill and not decay. The autopilot will correct the elliptical orbit to a circular one. The autopilot will then reset, allowing you to perform further maneuver. Should the lander come within the exosphere, it will switch to Descent and land. This mode is active in scenarios 0, 2 and 6.

### `ORBITAL_DESCENT`
The lander will attempt to land with the remaining fuel on the surface using a proportional gain controller. The autopilot may freeze for a moment as it tunes to the ideal value of  to be most fuel efficient or softest landing (version can be toggled using the `m` key, the default is fuel efficiency). This mode is active in scenarios 1, 4, 5, 7 and 8.

### `ORBITAL_DESCENT`
Input radius into the console and the lander will inject into an orbit of that radius. There is no fuel decay, and the autopilot will switch to Orbital Re-entry once it has reached a stable orbit. Active in scenario 3. `d` key will toggle between Orbital Injection and Orbit Descent.


## Additional Mechanics

### Wind
Toggling the `w` (default is off), there will be wind of a given average speed over a normal distribution (providing minor gusts now and then). The autopilot may eject the parachute if it is being dragged by it, and the user will be notified if this happens on the console.

### Engine Lag and Delay
An engine lag of 5s and delay of 2s can be toggled on or off using `c` and `v` key respectively. The autopilot will predict up to 5s ahead to try and counteract the effects of the delay, though it will not tune due to the time it takes to run through. Delay can cause inconsistent results in how successful a landing is, based on the timing of when in the simulation the autopilot is enabled.

### Miscellaneous Controls
The direction of the lander can be modified using the `x` and `z` keys to manually rotate it. The mechanics of the planet’s rotation is taken into account, so if the lander has 0 velocity it will still have a ground speed relative to Mars. Scenarios 7 and 8 were added to mimic 1 and 5, but give the lander 0 ground velocity at the start of the simulation.
