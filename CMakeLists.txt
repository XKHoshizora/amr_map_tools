cmake_minimum_required(VERSION 2.8.3)
project(amr_waypoint_tools)

add_compile_options(-pthread)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    rviz
    roscpp
    geometry_msgs
    std_msgs
    tf
    visualization_msgs
    interactive_markers
    move_base_msgs
    actionlib
    cmake_modules
)

add_service_files(FILES
    SaveWaypoints.srv
    AddNewWaypoint.srv
    GetNumOfWaypoints.srv
    GetWaypointByIndex.srv
    GetWaypointByName.srv
    GetChargerByName.srv
)

add_message_files(FILES
    Waypoint.msg
)

generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(TinyXML REQUIRED)
include_directories(${TinyXML_INCLUDE_DIRS})
link_directories(${TinyXML_LIBRARY_DIRS})

catkin_package(
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS include ${TinyXML_INCLUDE_DIRS}
    LIBRARIES amr_waypoint_tools
    DEPENDS TinyXML
)

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
    include/add_waypoint_tool.h
    src/add_waypoint_tool.cpp
    include/add_charger_tool.h
    src/add_charger_tool.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

# # Install rules
install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    waypoint_tools_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(waypoint_manager
    src/waypoint_manager.cpp
)
add_dependencies(waypoint_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_manager
    ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
)

add_executable(waypoint_editor
    src/waypoint_editor.cpp
)
add_dependencies(waypoint_editor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_editor
    ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
)

add_executable(waypoint_navi_test
    src/waypoint_navi_test.cpp
)
add_dependencies(waypoint_navi_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_navi_test
    ${catkin_LIBRARIES}
)

add_executable(waypoint_saver
    src/waypoint_saver.cpp
)
add_dependencies(waypoint_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_saver
    ${catkin_LIBRARIES}
)

add_executable(waypoint_navi_remote
    src/waypoint_navi_remote.cpp
    src/network/UDPServer.c
)
add_dependencies(waypoint_navi_remote ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_navi_remote
    ${catkin_LIBRARIES}
    pthread
)

add_executable(waypoint_navi_odom_report
    src/waypoint_navi_odom_report.cpp
    src/network/UDPClient.c
)
add_dependencies(waypoint_navi_odom_report ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_navi_odom_report
    ${catkin_LIBRARIES}
    pthread
)

add_executable(get_charger_position
    src/get_charger_position.cpp
)
add_dependencies(get_charger_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_charger_position
    ${catkin_LIBRARIES}
)

add_executable(waypoint_navi_server
    src/waypoint_navi_server.cpp
)
add_dependencies(waypoint_navi_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_navi_server
    ${catkin_LIBRARIES}
)

add_executable(amr_pose_navi_server
    src/pose_navi_server.cpp
)
add_dependencies(amr_pose_navi_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(amr_pose_navi_server
    ${catkin_LIBRARIES}
)