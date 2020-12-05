cmake_minimum_required(VERSION 2.8.3)
project(image_logger)

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    cv_bridge
)

catkin_python_setup() # added if there is a setup.py. In this case for the node

catkin_package(
    CATKIN_DEPENDS rospy std_msgs
)

# add_library(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE py)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE CMAKE_py_LINK_EXECUTABLE)
