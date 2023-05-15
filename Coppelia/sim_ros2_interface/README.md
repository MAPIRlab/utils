# ROS2 Interface plugin for CoppeliaSim

Supported ROS2 versions:

 - Humble Hawksbill
 - Foxy Fitzroy (in the 'foxy' branch)
 - Tested with Galactic (monroy)


### PreRequisites
1. Install Coppelia Simulator (from the official webpage)
2. Open CoppeliaSim from terminal (-h => headless)
    - Check that the ROS2 plugin was loaded (successfully).
    - Plugins are loaded when CoppeliaSim is launched (only on startup). 
    - Make sure to source the ROS2 environment prior to running CoppeliaSim (see .bashrc).
    - The plugin is now ready to be used.

    
### ERROR loading Plugin: Manual Compilation (only once)
If the plugin cannot be loaded, then you should recompile it by yourself. It is open source. Once compiled, copy the generated .so file to Coppelia directory.
- By default this package contains a COLCON_IGNORE as it only has to be compiled once.
- Remove the "COLCON_IGNORE" file
- Edit `meta/interfaces.txt` if you need to include more ROS interfaces. This pkg already contains laser abd Twist, but others can be added (see instructions below).
- Install xsltproc:
    sudo apt install xsltproc 
- Compile with:
    export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
    ulimit -s unlimited #otherwise compilation might freeze/crash
    colcon build --packages-select sim_ros2_interface --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
- Add the "COLCON_IGNORE" file to avoid further compilations
- Copy the library to Coppelia root directory
    ros_ws/build/sim_ros2_interface/libsimExtROS2.so --> CopeliaRoot Directory
- check that the plugin is loaded when CoppeliaSim starts


### Plugin Compilation Error: Update Interface and Compile (only once)
- simExtROS2 is the package that contains the ROS2 Interface that will be compiled to a ".so" file, and that is used by CoppeliaSim (loaded on startup).
- Download this pkg from the official GITHUB repository at (https://github.com/CoppeliaRobotics/simExtROS2) and follow instructions
- Compile with:
    export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
    ulimit -s unlimited #otherwise compilation might freeze/crash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
- It can take several minutes to compile
- Add the "COLCON_IGNORE" file to avoid further compilations
- That's it! The packages should have been generated and compiled to a library, which is automatically copied to the CoppeliaSim installation folder. 
- The plugin is now ready to be used.
