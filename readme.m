1. set the environment variable at ~/.bashrc as:
gedit ~/.bashrc 

2. comment the environment variable of HUSKY_GAZEBO_DESCRIPTION if you have claimed it:
#export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

3. using husky_custom_description as the HUSKY_DESCRIPTION value instead of raw package
sh $(rospack find husky_custom_description)/env-hooks/50.husky_custom_description.sh

4. set the correct path of official placeholder where the system could locate the entry of extension
export HUSKY_URDF_EXTRAS=$(rospack find husky_description)/urdf/empty.urdf

5. using husky_custom_gazebo as the HUSKY_GAZEBO_DESCRIPTION value instead of raw package
sh $(rospack find husky_custom_gazebo)/env-hooks/50.husky_custom_gazebo.sh