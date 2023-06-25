# KÄRCHER Coding Challenge

# Starting the ROS-Package "Kaercher
Please create a ROS-Workspace and copy the kaercher package in the workspace. Build the workspace with **catkin_build** or **catkin_make**.  Then sourcing the workspace. The package have five directories:
* config: this directory contains the json_file
* include: here is the header-file *cleaning_robot.h*
* launch: There is the launch-file, which starting the programm
* src: there is mostly the source-code for the project
* test: there is the ctest environment

To start the launch-file: 
**roslaunch kaercher kaercher.launch**
The roslaunch-file contains a parameter, which is the json-file for the calculation for the whole distance, the cleaned area and the total_time of the path.

# CTest
for the ctest, you must going in the ros-package. Then execute the following commands:
**cmake -S. -Bbuild** 

After that go in the build-directory and build the whole code with **cmake --build .**. After that you can make the ctest with execute the command **ctest**