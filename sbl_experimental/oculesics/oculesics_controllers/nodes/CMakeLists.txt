rosbuild_add_executable( random_oculesics_controller random_oculesics_controller.cpp )
rosbuild_add_compile_flags( random_oculesics_controller -std=c++0x )
target_link_libraries( random_oculesics_controller ${PROJECT_NAME} )

rosbuild_add_executable( probabilistic_oculesics_controller probabilistic_oculesics_controller.cpp )
rosbuild_add_compile_flags( probabilistic_oculesics_controller -std=c++0x )
target_link_libraries( probabilistic_oculesics_controller ${PROJECT_NAME} )
