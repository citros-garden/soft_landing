# General Info

This is a ROS simulation of soft landing.
In the ROS system we have two nodes: the first represents the dynamics and the second one is the controller.

# System dynamics





# The controller




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

4. build and source the workspace:
	```sh
	colcon build
	source install/local_setup.bash
	```


# Run the default example: