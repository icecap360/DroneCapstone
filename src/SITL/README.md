# SITL

Software in The Loop (SITL) provides a virtual environment to test the functionality of the drone.  
Testing environment is Ubuntu 18.04 with ROS Melodic.  

### Dependencies  
- [Ardupilot 4.0.3](https://github.com/ArduPilot/ardupilot/tree/Copter-4.0.3)  
	- cherry pick bugfix commit f504009287221c7fda9cfa9c389a13af702aa9fd  
- Gazebo 9  
- [Ardupilot-Gazebo Plugin](https://github.com/khancyr/ardupilot_gazebo)

### Launching Sample Simulation
1. Nagivate to `ardupilot_gazebo/worlds` directory (from Ardupilot-Gazebo Plugin repo).
2. `rosrun gazebo_ros gazebo --verbose iris_ardupilot.world`
3. Load SITL environment from Ardupilot.
	- `sim_vehicle.py -f gazebo-iris --console --map`  
4. Navigate to `DroneCapstone/src/SITL/src/launch/`.
5. Launch mavros.
	- `roslaunch apm.launch`
