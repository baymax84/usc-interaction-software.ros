add_library_auto( ${PROJECT_NAME} quickdev.cpp )

target_link_libraries( ${PROJECT_NAME} pthread boost_thread )
