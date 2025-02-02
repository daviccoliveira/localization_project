# localization_project


Install catkin build tools

sudo apt-get install python3-catkin-tools

Clone lar_ufba repo

https://github.com/lar-deeufba/lar_gazebo/tree/noetic

Install apriltag 

git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
cd ~/catkin_ws                          # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
catkin build    # Build all packages in the workspace (catkin_make_isolated will work also)


Install deps

rosdep init
rosdep update
rosdep install --from-paths src/lar_gazebo --ignore-src -r -y --rosdistro noetic


