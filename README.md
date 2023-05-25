# outdoor_gcs
This is the qt ground control station for drone with pixhawk mounted and ground station running mavros.

Please read **readme_file.pdf** for more details.

```
Git clone and catkin build

roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0

rosrun outdoor_gcs outdoor_gcs 
```

This branch is created for multi-uav with px4_command running onboard

Please refer to https://github.com/LonghaoQian/px4_command or https://github.com/chenc159/px4_command
