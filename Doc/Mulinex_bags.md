# Guide to Open Mulinex .db3 bags in MATLAB

Ensure to work in the ros2 workspace folder `ros2_ws/`.

## Import custom messages in Matlab
If you have the `pi3hat_moteus_int_msgs` package installed just run in `MATLAB`:

```matlab
ros2genmsg("src/")
```
otherwise downolad the package from this [repo](https://github.com/CentroEPiaggio/Pi3hat_robotic_systems), copy it in `ros2_ws/src/` and run in a terminal:

```bash
 colcon build --symlink-install && . install/setup.bash
 ```

from `ros2_ws/`. Now run `ros2genmsg("src/")` in `MATLAB`.
To check if everything worked run `ros2 msg list` in `MATLAB` and look for 
```
pi3hat_moteus_int_msgs/JointsCommand
pi3hat_moteus_int_msgs/JointsStates
pi3hat_moteus_int_msgs/OmniMulinexCommand
pi3hat_moteus_int_msgs/PacketPass
```

## Read the bags

To avoid problem put the bags folder in `ros2_ws/bags/` then:

```matlab
bags = ros2bagreader("path/to/bag")
```

If it returns an error regardin `libstdc++.so.6` close all `MATLAB` windows, open a terminal and run:

```bash
export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libcurl.so"
```