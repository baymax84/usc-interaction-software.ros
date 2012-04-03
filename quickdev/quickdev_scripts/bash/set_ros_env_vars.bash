#!/bin/bash
local_path=`rospack find quickdev_scripts`/bash/
scripts=`ls $local_path | grep "set_ros" | grep -v "set_ros_env_vars.bash"`
for script in $scripts
do
  source $local_path$script $@;
done
