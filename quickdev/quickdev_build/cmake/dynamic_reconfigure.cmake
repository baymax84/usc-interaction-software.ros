macro( include_dynamic_reconfigure )
	rosbuild_find_ros_package( dynamic_reconfigure )
	include( ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake )
endmacro( include_dynamic_reconfigure )

macro( quickdev_gencfg )
	include_dynamic_reconfigure()
	gencfg()
endmacro( quickdev_gencfg )
