cmake_minimum_required(VERSION 3.8)
project(lwa4p_gazebo)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf config launch worlds DESTINATION share/${PROJECT_NAME})

# simulation plumbing (always installed)
install(PROGRAMS scripts/finger_mimic.py scripts/gz_cloud_frame.py
        DESTINATION lib/${PROJECT_NAME})

# any other scripts/*.py you add locally are installed too (git ignores them,
# see .gitignore) - run them with `ros2 run lwa4p_gazebo <name>.py`
file(GLOB LOCAL_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
list(REMOVE_ITEM LOCAL_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/finger_mimic.py
                               ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gz_cloud_frame.py)
if(LOCAL_SCRIPTS)
  install(PROGRAMS ${LOCAL_SCRIPTS} DESTINATION lib/${PROJECT_NAME})
endif()
ament_package()
