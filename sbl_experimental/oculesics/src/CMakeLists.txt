#rosbuild_add_executable( gaze_controller gaze_controller.cpp )

#rosbuild_add_executable( test_pid test_pid.cpp )
#rosbuild_add_executable( test_strain_pid test_strain_pid.cpp )
#rosbuild_add_executable( test_mpc test_mpc.cpp )
#rosbuild_add_executable( test_mpc_indices test_mpc_indices.cpp )

rosbuild_add_boost_directories()
rosbuild_add_library( ${PROJECT_NAME} #gaze_chain_core.cpp
									  oculesics_core.cpp )									  
rosbuild_add_compile_flags( ${PROJECT_NAME} -std=c++0x )
rosbuild_link_boost( ${PROJECT_NAME} math_tr1 )
									  
rosbuild_add_executable( random_oculesics_controller random_oculesics_controller.cpp )
rosbuild_add_compile_flags( random_oculesics_controller -std=c++0x )
target_link_libraries( random_oculesics_controller ${PROJECT_NAME} )

rosbuild_add_executable( probabilistic_oculesics_controller probabilistic_oculesics_controller.cpp )
rosbuild_add_compile_flags( probabilistic_oculesics_controller -std=c++0x )
target_link_libraries( probabilistic_oculesics_controller ${PROJECT_NAME} )

#target_link_libraries( test_pid ${PROJECT_NAME} )
#target_link_libraries( test_strain_pid ${PROJECT_NAME} )
#target_link_libraries( test_mpc ${PROJECT_NAME} )
#target_link_libraries( test_mpc_indices ${PROJECT_NAME} )
