# General Info

This is a ROS simulation of soft landing of an object.
In the ROS system we have two nodes: the first represents the dynamics and the second one is the controller.

![Screenshot from 2023-04-16 11-20-43](https://user-images.githubusercontent.com/114152002/232285879-d070707d-d2ef-4f31-ba97-00c10216c572.png)

# System dynamics
The system's equation of motion is the kinematic equation of a free body fall:

$$ v = \int \left ( u-g \right )dt +v_0 $$


$$ r = \int \left ( v \right )dt +r_0 $$

when `u` is the controller fedback , `g` is the gravity vector `r0` , `v0` is the initial conditions of the object.



# The controller
Because we simulate a soft landing, we want that our object to land with the minimum velocity (v_f--->0) at the landing point `u0` we want.
the controller is based on this paper:

*S. Gutman, "Rendezvous and Soft Landing in Closed Form via LQ Optimization," 2019 27th Mediterranean Conference on Control and Automation (MED), Akko, Israel, 2019, pp. 536-540, doi: 10.1109/MED.2019.8798572.*





# Installation

1. Clone the repository:
   ```sh
    git@github.com:citros-garden/soft_landing.git
   ```

2. open the repository in the VScode:
	```sh
	cd ~/soft_landing
	code .
	```
3. open the repository in the container from VScode with `reopen in container` option.

4. source and build:
	```sh
	colcon build
	source install/local_setup.bash
	```

# Run the default example
Run the example of an object with the initial condition: 

$$\overrightarrow{r_0} =[2000,1000,4000][m]$$

$$\overrightarrow{v_0} = [-10,2,-50] [m/s]$$

And the goal is to land in the point:

$$u_0=[0,0,0]$$ 

On the moon,so to gravity is: $\overrightarrow{g}=[0,0,1.62] [m/sec^2]$.

Open [Foxglove](https://foxglove.dev/) to view a graphical representation of the simulation.

In Foxglove add data source: click on Open connection,choose the Rosbridge and in the URL write: ws://localhost:9090 and then click open.

Then load the soft landing layout: `soft landing layout.json` layout file.


Run the launch file:

```sh
ros2 launch dynamics dynamics_controller.launch.py
```

## Production Docker
```bash
docker build -t soft_landing .

citros docker-login
docker tag soft_landing us-central1-docker.pkg.dev/citros/lulav/soft_landing
docker push us-central1-docker.pkg.dev/citros/lulav/soft_landing
```