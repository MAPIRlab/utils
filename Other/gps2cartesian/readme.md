# gps2cartesian
Simple ROS2 node to convert GPS (geodetic) --> cartesian (UTM/lat0long0) --> map (local ref system)
It does not account for orientation as it only relies on the GPS NavFix data