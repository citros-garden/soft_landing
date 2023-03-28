# General Info

This is a ROS simulation of soft landing of an object.
In the ROS system we have two nodes: the first represents the dynamics and the second one is the controller.



# System dynamics
The system's equation of motion is the kinematic equation of a free body fall:

$$ v = \int \left ( u-g \right )dt +v_0 $$
$$ r = \int \left ( v \right )dt +r_0 $$

when `u` is the controller fedback , `g` is the gravity vector `r0` , `v0` is the initial conditions of the object.



# The controller
Because we simulate a soft landing, we want that our object to land with the minimum velocity (v_f--->0) at the landing point `u0` we want.
?





# Installation

1. Clone the repository:
   ```sh
    git@github.com:lulav/citros_soft_landing.git
   ```

2. open the repository in the VScode:
	```sh
	cd ~/citros_soft_landing
	code.
	```
3. open the repository in the container from VScode with `reopen in container` option.

4. source the workspace:
	```sh
	
	source install/local_setup.bash
	```


# Run the default example:
Run the example of an object with the initial condition: r0 =[10,0,100] [meter] and v0 = [0,0,0] [meter/sec] that we want to land in the point u0=[0,0,0] on the moon g=[0,0,1.62] [meter/sec^2].

first, build the nodes:
```sh
     colcon build
```

then,run the launch file:
```sh
    ros2 launch dynamics dynamics_controller.launch.py
```

####add simulation in foxglove