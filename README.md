# emprise_joint_control

RUN SERVER:
```
conda activate py310
cd ~/feeding-deployment/src/feeding_deployment/control/robot_controller
python arm_server.py 
```

RUN CLIENT & CONTROLLER:
```
conda activate py310
cd ~/catkin_ws
source devel/setup.bash
roslaunch joint_controller_pkg joint_controller.launch
```

If system does not have USB port access run: ``sudo chmod a+rw /dev/ttyUSB0``

Parameters:
 - robot_mode: sim | kinova
 - controller: pos
 - input_mode: test | key | exo
 - rev: front | back (forward direction of the exoskeleton; see front/back markings on base; defaults to front)


Exoskeleton servos:
- configured via Dynamixel Wizard 2.0 to have ids 001-007; base motor is 1
- baud rate: 4M
- protcol 2.0
