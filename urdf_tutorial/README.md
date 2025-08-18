In this package:

1. We have designed pikachu arm using solidworks.
2. Then we used SW urdf plugin to emit corresponding meshes and urdf package.
3. We then copied the meshes and urdf files for the arm into this package.
4. joint2 and joint5 is fully controlled by controllers.
5. Does not use moveit. Target angles for both the joints comes from test_forward_position_controller.launch.py


Commands:
Windows does not share the serial ports with wsl by default. Use  https://learn.microsoft.com/en-us/windows/wsl/connect-usb to share the serial port with WSL. More instruction in "ubuntu on windows and ros notes" doc.
I downloaded the arduino binary here:  ~/arduino/arduino-ide_2.3.6_Linux_64bit/arduino-ide

Open the arduino ide from within the wsl.
Close any arduino IDE first to release the serial port.
colcon build  --packages-select urdf_tutorial; source install/setup.bash; ros2 launch urdf_tutorial kp.display.launch.py;
ros2 launch urdf_tutorial test_forward_position_controller.launch.py;
