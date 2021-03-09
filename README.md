# spacenav_rviz

Control your rviz view with a [3Dconnexion SpaceMouse](https://3dconnexion.com/us/product/spacemouse-wireless/).

Tested with SpaceMouse Wireless, but should work with SpaceMouse Compact and
maybe SpaceMouse Pro.


## Installation

```console
# Use your ROS distro instead of noetic
$ sudo apt install ros-noetic-spacenav-node
```


## Usage

For a quick demo, run:

```console
$ roslaunch spacenav_rviz rviz.launch
```

To use it in your launch file, change:

```xml
<node name="rviz" pkg="rviz" type="rviz" args="-d your_config.rviz" />
```

to

```xml
<include file="$(find spacenav_rviz)/launch/rviz" >
  <arg name="config" value="your_config.rviz" />
</include>
```

In rviz "Views" panel, set:
- "Type" to `FrameAligned`
- "Target Frame" to `camera`
- "Point towards" to `+x axis`

You may need to click "Zero" to reset the view.

Tilt your SpaceMouse to control the camera position and orientation. Press the
right button to speed up and the left button to slow down.

## License

MIT


## Author

Naoki Mizuno (naoki.mizuno.256@gmail.com)
