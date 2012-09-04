rosbuild_add_executable( openni_multitracker openni_multitracker.cpp )
rosbuild_add_executable( person_auto_selector person_auto_selector.cpp )
rosbuild_add_compile_flags( person_auto_selector -std=c++0x )
