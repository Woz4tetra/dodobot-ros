cmake_minimum_required(VERSION 2.8.3)
project(db_chassis)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
    tf
    nav_msgs
    dynamic_reconfigure
)

catkin_python_setup() # added if there is a setup.py. In this case for the node

add_message_files(
    FILES
    LinearPosition.msg
    LinearVelocity.msg
)

add_service_files(
    FILES
    DodobotOdomReset.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/DodobotChassis.cfg
)

catkin_package(
    CATKIN_DEPENDS rospy std_msgs message_runtime
)

# add_library(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE py)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE CMAKE_py_LINK_EXECUTABLE)
