#rosbuild_add_executable( gaze_controller gaze_controller.cpp )

#rosbuild_add_executable( test_pid test_pid.cpp )
#rosbuild_add_executable( test_strain_pid test_strain_pid.cpp )
#rosbuild_add_executable( test_mpc test_mpc.cpp )
#rosbuild_add_executable( test_mpc_indices test_mpc_indices.cpp )

rosbuild_add_boost_directories()
rosbuild_add_library( ${PROJECT_NAME} oculesic_policy.cpp
                                      param_array_reader.cpp
                                      point3d.cpp
                                      probabilistic_oculesic_policy.cpp
                                      random_oculesic_policy.cpp
                                      oculesic_controller.cpp )

rosbuild_link_boost( ${PROJECT_NAME} math_tr1 )

#target_link_libraries( test_pid ${PROJECT_NAME} )
#target_link_libraries( test_strain_pid ${PROJECT_NAME} )
#target_link_libraries( test_mpc ${PROJECT_NAME} )
#target_link_libraries( test_mpc_indices ${PROJECT_NAME} )
