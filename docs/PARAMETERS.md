### Parameters

 Name         |  Description |  Default
--------------|--------------|----------
`dev`         |  Device path of robot |  `/dev/ttyUSB0`
`base_frame`  |  The robot's base frame ID | `base_footprint`
`odom_frame`  |  The robot's odometry frame ID | `odom`
`latch_cmd_duration` | If this many seconds passes without receiving a velocity command the robot stops | `0.2`
`loop_hz`     |  Frequency of internal update loop |  `10.0`
`publish_tf`  |  Publish the transform from `odom_frame` to `base_frame` | `true`  
`robot_model` |  The type of robot being controlled (supported values: `ROOMBA_400`, `CREATE_1` and `CREATE_2`) | `CREATE_2`
`baud`        |  Serial baud rate | Inferred based on robot model, but is overwritten upon providing a value

### Publishers

 Topic       | Description  | Type
-------------|--------------|------
 `battery/capacity` | The estimated charge capacity of the robot's battery (Ah) | [std_msgs/Float32][float32]
 `battery/charge` | The current charge of the robot's battery (Ah) | [std_msgs/Float32][float32]
 `battery/charge_ratio` | Charge / capacity | [std_msgs/Float32][float32]
 `battery/charging_state` | The chargins state of the battery | [ca_msgs/ChargingState][chargingstate_msg]
 `battery/current` | Current flowing through the robot's battery (A). Positive current implies charging | [std_msgs/Float32][float32]
 `battery/temperature` | The temperature of the robot's battery (degrees Celsius) | [std_msgs/Int16][int16]
 `battery/voltage` | Voltage of the robot's battery (V) | [std_msgs/Float32][float32]
 `bumper` | Bumper state message (including light sensors on bumpers) | [ca_msgs/Bumper][bumper_msg]
 `clean_button` | 'clean' button is pressed ('play' button for Create 1) | [std_msgs/Empty][empty]
 `cliff` | 'cliff' sensors are pressed | [ca_msgs/Cliff][cliff_msg]
 `day_button` |  'day' button is pressed | [std_msgs/Empty][empty]
 `hour_button` | 'hour' button is pressed | [std_msgs/Empty][empty]
 `minute_button` | 'minute' button is pressed | [std_msgs/Empty][empty]
 `dock_button` | 'dock' button is pressed ('advance' button for Create 1) | [std_msgs/Empty][empty]
 `spot_button` | 'spot' button is pressed | [std_msgs/Empty][empty]
 `ir_omni` | The IR character currently being read by the omnidirectional receiver. Value 0 means no character is being received | [std_msgs/UInt16][uint16]
 `joint_states` | The states (position, velocity) of the drive wheel joints | [sensor_msgs/JointState][jointstate_msg]
 `mode` | The current mode of the robot (See [OI Spec][oi_spec] for details)| [ca_msgs/Mode][mode_msg]
 `odom` |  Robot odometry according to wheel encoders | [nav_msgs/Odometry][odometry]
 `wall` | Wall is detected | [std_msgs/Bool][bool]
 `wheeldrop` | At least one of the drive wheels has dropped | [std_msgs/Empty][empty]
 `/tf` | The transform from the `odom` frame to `base_footprint`. Only if the parameter `publish_tf` is `true` | [tf2_msgs/TFMessage](tf_msg)

### Subscribers

Topic       | Description   | Type
------------|---------------|------
`cmd_vel` | Drives the robot's wheels according to a forward and angular velocity | [geometry_msgs/Twist][twist]
`debris_led` | Enable / disable the blue 'debris' LED | [std_msgs/Bool][bool]
`spot_led`   | Enable / disable the 'spot' LED | [std_msgs/Bool][bool]
`dock_led`   | Enable / disable the 'dock' LED | [std_msgs/Bool][bool]
`check_led`  | Enable / disable the 'check robot' LED | [std_msgs/Bool][bool]
`power_led`  | Set the 'power' LED color and intensity. Accepts 1 or 2 bytes, the first represents the color between green (0) and red (255) and the second (optional) represents the intensity with brightest setting as default (255) | [std_msgs/UInt8MultiArray][uint8multiarray]
`set_ascii` | Sets the 4 digit LEDs. Accepts 1 to 4 bytes, each representing an ASCII character to be displayed from left to right | [std_msgs/UInt8MultiArray][uint8multiarray]
`dock` | Activates the demo docking behaviour. Robot enters _Passive_ mode meaning the user loses control (See [OI Spec][oi_spec]) | [std_msgs/Empty][empty]
`undock` | Switches robot to _Full_ mode giving control back to the user | [std_msgs/Empty][empty]
`define_song` | Define a song with up to 16 notes. Each note is described by a MIDI note number and a float32 duration in seconds. The longest duration is 255/64 seconds. You can define up to 4 songs (See [OI Spec][oi_spec]) | [ca_msgs/DefineSong][definesong_msg]
`play_song` | Play a predefined song | [ca_msgs/PlaySong][playsong_msg]

[bool]:  http://docs.ros.org/api/std_msgs/html/msg/Bool.html
[bumper_msg]:  https://github.com/RoboticaUtnFrba/create_autonomy/blob/kinetic-devel/ca_msgs/msg/Bumper.msg
[chargingstate_msg]:  https://github.com/RoboticaUtnFrba/create_autonomy/blob/kinetic-devel/ca_msgs/msg/ChargingState.msg
[cliff_msg]:https://github.com/RoboticaUtnFrba/create_autonomy/blob/kinetic-devel/ca_msgs/msg/Cliff.msg
[definesong_msg]:  https://github.com/RoboticaUtnFrba/create_autonomy/blob/kinetic-devel/ca_msgs/msg/DefineSong.msg
[empty]:  http://docs.ros.org/api/std_msgs/html/msg/Empty.html
[float32]:  http://docs.ros.org/api/std_msgs/html/msg/Float32.html
[int16]:  http://docs.ros.org/api/std_msgs/html/msg/Int16.html
[jointstate_msg]:  http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
[mode_msg]:  https://github.com/RoboticaUtnFrba/create_autonomy/blob/kinetic-devel/ca_msgs/msg/Mode.msg
[odometry]:  http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
[oi_spec]:  https://www.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf
[playsong_msg]:  https://github.com/RoboticaUtnFrba/create_autonomy/blob/kinetic-devel/ca_msgs/msg/PlaySong.msg
[tf_msg]: http://docs.ros.org/jade/api/tf2_msgs/html/msg/TFMessage.html
[twist]:  http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[uint16]:  http://docs.ros.org/api/std_msgs/html/msg/UInt16.html
[uint8multiarray]:  http://docs.ros.org/api/std_msgs/html/msg/UInt8MultiArray.html
