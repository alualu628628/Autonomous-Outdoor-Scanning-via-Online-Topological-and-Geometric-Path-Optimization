#source ./devel/setup.bash
#export HUSKY_DESCRIPTION=~/ros/sensorplus/src/husky_description/urdf/description.xacro
#export HUSKY_DESCRIPTION=~/ros/myHusky/src/husky_description/urdf/description.xacro
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_custom_gazebo)/urdf/custom_description.gazebo.xacro
#sh $(rospack find husky_custom_description)/env-hooks/50.husky_custom_description.sh
export HUSKY_DESCRIPTION=$(rospack find husky_custom_description)/urdf/custom_description.gazebo.xacro
#export HUSKY_URDF_EXTRAS=$(rospack find husky_custom_description)/urdf/empty.urdf
#sh $(rospack find husky_custom_gazebo)/env-hooks/50.husky_custom_gazebo.sh
#export GAZEBO_MODEL_PATH=$GAZEBO_RESOURCE_PATH:~/model_editor_models:~/.gazebo/models:~/ros/Husky_Simulation/src/husky_custom_gazebo/models
export GAZEBO_MODEL_PATH=~/ros/Husky_Simulation/src/husky_custom_gazebo/models
