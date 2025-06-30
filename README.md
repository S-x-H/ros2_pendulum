# Simple Pendulum Simulation

Given the state-space equation of a simple pendulum given below:

$$
\dot{\theta}_1 = \theta_2
$$

$$
\dot{\theta}_2 = -\frac{g}{l} \sin(\theta_1) + \frac{1}{ml^2} \tau
$$

Where:
- $\theta_1$, $\theta_2$ are the system states angular position $\theta_1$ and angular velocity $\theta_2$
- $m$ is the point mass at the end of the pendulum
- $l$ is the length the pendulum
- $\tau$ is the torque applied
- $g$ is ccceleration due to gravity

This ROS2 Kilted package simulates a system using a Forward-Euler integration scheme with a time-step of 1ms (pendulum rest state at 0).

## Implementation

The data flow begins at the 'clock' node, which publishes a message with the current timestamp approximately once per update step (1ms). This timestamp is used in synchronisation of subsequent dataflow, and propagated throughout the other nodes. Each node that receives a clock tick takes on the tick timestamp as its own timestamp, and does not repoll the current time.

If in use (i.e. in [Example 2](#example-2)), the 'torque' node receives the clock tick first, and publishes a corresponding torque for each time increment in accordance with the logic given. Because in this implementation, the ticking of the clock is not exactly precise to the requested update rate, the modulo in the torque logic has been replaced with an approximate modulo of appropriate precision (time step / 2).

The main state simulation is performed in the 'pendulum' node, which recieves the clock tick and optionally the corresponding torque. This node performs the state update with the correct delta (current time - last update time), but does not necessarily publish every update round. Because the update step is 1ms but the state publication was only requested at 500Hz, the current configuration has this node making publications every 2 updates.

Finally, the plotter receives the position, velocity, and optionally torque, synchronised to the same timestamp.

### Units

Simulation mathematics are calculated in radians, though the inputs of initial position and velocity are accepted in degrees for easier human readability. The plotter outputs data in radians by default but can be configured to output degrees instead.

### Messages

The torque, position, and velocity are published as 'Temperature' sensor messages. Despite not being relevantly named, the 'Temperature' message contains the desired fields for this application (a timestamp and a single float).

## Examples

The following examples are provided, both with the fixed values:
- $m = 0.5kg$
- $l = 0.2m$


### Example 1

Launchable with:

```
ros2 launch pendulum example1.launch.py
```

which is directly equivalent to:

```
ros2 launch pendulum pendulum.launch.py duration:=3 log_name:=example1 pos_0:=60 const_torque:=0 plot_degrees:=true
```

The `example1` launch file runs the simulation for 3 seconds, without additionally applied torque ($\tau = 0Nm$), with the initial conditions:
 - $\theta_1 = 60$
 - $\theta_2 = 0$

### Example 2

Launchable with:

```
ros2 launch pendulum example2.launch.py
```

which is directly equivalent to:

```
ros2 launch pendulum pendulum.launch.py duration:=8 log_name:=example2 plot_degrees:=true
```

The `example2` launch file runs the simulation for 8 seconds, with the initial conditions:
 - $\theta_1 = 0$
 - $\theta_2 = 0$

And the system actuated with torque as so:
 - when $t \% 1 = 0$: $\tau = 1Nm$
 - when $(t - 0.5) \% 1 = 0$: $\tau = -1Nm$
 - otherwise: $\tau = 0Nm$

#### Notes

Because the state updates occur at 1000Hz but only publish at 500Hz, sometimes the changing value of the torque does not appear in the result output. Because the torque is equal to 1Nm or -1Nm for only a single clock tick, it may be the tick that is not published and therefore not graphed. However, the corresponding effect of the torque on the rest of the values is clearly visible.

### Result Verification

For comparison to the ROS2 system, the same mathematical simulation was also computed in an excel spreadsheet, with outputs in radians. This is given in test.xlsx.

Example outputs of the above scenarios are provided in the `results` folder, in csv format in both radians and degrees, along with screenshots of each visualised in plotjuggler. The radian plots can be compared against the graphs generated in excel.