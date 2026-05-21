# Code for BPC-PRP course

[České README](https://github.com/Drochec/bpc-prp-drochecmara/blob/main/README.cs.md)

Template from: [Robotics-BUT](https://github.com/Robotics-BUT/bpc-prp-cpp-project-template/tree/53c2f887439a6ac508bb4e62157084f8cd38cab8)

Folder organization:
- algorithms/ - Used algorithms (PID, tag detection,..)
- loops/ - Control loops 
- nodes/ - ROS2 nodes for devices on the robot

The project doesn't contain a launch file as is standard in ROS2 projects, instead all nodes are launched in the main() function.

Run the project using the command:
`ros2 run prp_project main`

The project uses the ROS2 (Humble) framework, before working with this project we strongly recommend familiarising yourself with it [ROS2 Humble](https://docs.ros.org/en/humble/index.html)

Authors:
- Jan Drochýtek (256278)
- Marek Holečka (256294)
