# Manipulation
### Visualiye the robot in RViz
```bash
   ros2 launch ainex_description display.launch.py gui:=true/false
```

### Lock/Unlock all joints of the robot
```bash
   ros2 service call /Lock_All_Joints std_srvs/srv/Empty {}
   ros2 service call /Unlock_All_Joints std_srvs/srv/Empty {}
```