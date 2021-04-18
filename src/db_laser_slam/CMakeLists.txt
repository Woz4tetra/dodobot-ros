cmake_minimum_required(VERSION 2.8.3)
project(db_laser_slam)

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    std_msgs
    genmsg
    message_generation
)

catkin_python_setup() # added if there is a setup.py. In this case for the node

# ## Generate messages in the 'msg' folder
# add_message_files(
#     FILES
# )
#
# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )
# add_library(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE py)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE CMAKE_py_LINK_EXECUTABLE)


catkin_package(
    CATKIN_DEPENDS rospy std_msgs message_runtime
)

# catkin_install_python(PROGRAMS
#   src/db_laser_slam_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )