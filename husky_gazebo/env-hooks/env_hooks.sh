## mallan: This adds our meshes directory to Gazebo's search path, but
## there has got to be a better way than setting environment variables to do this
## FIXME: do this without setting environment variables!
export GAZEBO_MODEL_PATH="$(catkin_find --first-only husky_gazebo worlds):${GAZEBO_MODEL_PATH}"

export GAZEBO_RESOURCE_PATH=
export GAZEBO_RESOURCE_PATH="$(catkin_find --first-only husky_gazebo worlds):/usr/share/gazebo-2.2:/usr/share/gazebo_models:${GAZEBO_RESOURCE_PATH}"
