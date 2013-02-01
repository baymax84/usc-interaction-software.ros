rosbuild_add_executable( openni_multitracker openni_multitracker.cpp )
target_link_libraries( openni_multitracker ${OPENNI_LIBRARIES} )
rosbuild_add_executable( person_auto_selector person_auto_selector.cpp )
rosbuild_add_compile_flags( person_auto_selector -std=c++0x )
