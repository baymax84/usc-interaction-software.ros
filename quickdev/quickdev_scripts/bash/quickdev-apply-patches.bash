#!/bin/bash

# replace roscpp msg_gen.py with our "better" version
roscpp_path=`rospack find roscpp`
echo "Found roscpp at $roscpp_path"
quickdev_scripts_path=`rospack find quickdev_scripts`
echo "Found quickdev_scripts at $quickdev_scripts_path"

echo "Backing up roscpp's msg_gen.py and linking to quickdev's msg_gen.py"
cd $roscpp_path/src/roscpp &&
sudo mv msg_gen.py bk.msg_gen.py &&
sudo rm msg_gen.pyc &&
sudo ln -s $quickdev_scripts_path/python/msg_gen.py

echo "All done!"
