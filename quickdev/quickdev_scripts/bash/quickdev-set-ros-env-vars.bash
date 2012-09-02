#!/bin/bash
local_path=`rospack find quickdev_scripts`/bash/
scripts=`ls $local_path | grep "quickdev-set-ros" | grep -v "quickdev-set-ros-env-vars.bash"`
for script in $scripts
do
  source $local_path$script $@;
done
