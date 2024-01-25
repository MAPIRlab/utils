# Coppelia Pkgs
This folder holds a series of pkgs to make life easier when working with the Robotics Simulator [CoppeliSIM](https://www.coppeliarobotics.com/)

The version of Coppelia used to develop these packages is 4.5.1_rev4. We cannot guarantee that they will be compatible with later releases of Coppelia.

## Installation and Configuration
To use this pkgs, you need to install the simulator on your computer. See instructions [here](https://www.coppeliarobotics.com/downloads)

Then, you need to set the **COPPELIASIM_ROOT_DIR** Env Variable with the installation directory path. To do that, add to your ~/.bashrc config file a line like this:
```
export COPPELIASIM_ROOT_DIR='Full_path_to_Coppelia_Folder'
```
Note: Do not use ~ in the full path.
