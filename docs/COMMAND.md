# Commanding your Create

You can move the robot around by sending [geometry_msgs/Twist][twist] messages to the topic `cmd_vel`:

```bash
linear.x  (+)     Move forward (m/s)
          (-)     Move backward (m/s)
angular.z (+)     Rotate counter-clockwise (rad/s)
          (-)     Rotate clockwise (rad/s)
```

## Velocity limits

`-0.5 <= linear.x <= 0.5` and `-4.25 <= angular.z <= 4.25`

## Teleoperation

`ca_tools` comes with a launch file for teleoperating Create with a joystick.

``` bash
roslaunch ca_tools joy_teleop.launch [joy_config:=xbox360]
```

There exists configuration files for the [Xbox 360 wired controller](https://www.amazon.ca/Microsoft-Xbox-360-Wired-Controller/dp/B003ZSN600) and the [Logitech F710 controller](http://gaming.logitech.com/en-ca/product/f710-wireless-gamepad). You can adapt these files for your preferred joystick configuration.
