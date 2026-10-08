cmake_minimum_required(VERSION 3.8)
project(lwa4p_gazebo)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf config launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/go_to.py scripts/pick_tube.py scripts/simple_moveit.py
        scripts/finger_mimic.py scripts/bin_pick.py
        DESTINATION lib/${PROJECT_NAME})
ament_package()
