# localization_project


### Install catkin build tools

sudo apt-get install python3-catkin-tools

### Clone all repos

git clone https://github.com/lar-deeufba/lar_gazebo.git
cd lar_gazebo
git checkout noetic
cd ..

git clone https://github.com/AprilRobotics/apriltag.git
git clone https://github.com/AprilRobotics/apriltag_ros.git

### Install deps

rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y 

### Build

cd ~/<catkin_ws>
catkin build