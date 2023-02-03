# SITL

Software in The Loop (SITL) provides a virtual environment to test the functionality of the drone.  
Testing environment is Ubuntu 18.04 with ROS Melodic.  

### Dependencies  
- [Ardupilot 4.0.3](https://github.com/ArduPilot/ardupilot/tree/Copter-4.0.3)  
	- cherry pick bugfix commit f504009287221c7fda9cfa9c389a13af702aa9fd  
- Gazebo 9  
- [Ardupilot-Gazebo Plugin](https://github.com/SwiftGust/ardupilot_gazebo)
- [gStreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=c#install-gstreamer-on-ubuntu-or-debian)

### Setup  
1. Follow setup instructions in ardupilot_gazebo repo.  
2. Replace `~/.gazebo/models/gimbal_small_2d/model.sdf` with one provided in this repo in `models/gimbal_small_2d'.  
3. Ensure plugins inside build folder are visible to Gazebo, either by moving plugins to default Gazebo plugin path or adding build folder to $GAZEBO_PLUGIN_PATH. 
4. Setup IP address of host machine within `~/.gazebo/models/gimbal_small_2d/model.sdf` under `libgazebo_gst_camera_plugin.so` plugin parameters.  

### Launching Sample Simulation
1. Load SITL environment from Ardupilot.
      - `sim_vehicle.py -f gazebo-iris --console --map`
2. Navigate to `DroneCapstone/src/SITL/src/`. 
3. `rosrun gazebo_ros gazebo --verbose worlds/iris_cam_ardupilot.world`  
4. Launch mavros.
	- `roslaunch launch/apm.launch`
5. View gStreamer output on external device.
	- `gst-launch-1.0 -e -v udpsrc port=5600 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! fpsdisplaysink sync=false text-overlay=false`
