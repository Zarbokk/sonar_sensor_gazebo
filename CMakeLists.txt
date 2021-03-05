cmake_minimum_required(VERSION 3.0.2)
project(sonar_sensor_gazebo)

find_package(catkin REQUIRED gazebo_dev gazebo_plugins gazebo_ros roscpp std_msgs)
find_package(gazebo REQUIRED)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${GAZEBO_MSG_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


catkin_package(INCLUDE_DIRS
        CATKIN_DEPENDS
        gazebo_dev
        gazebo_plugins
        gazebo_ros)


install(DIRECTORY launch meshes rviz urdf world
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )


add_library(rotating_sonar_plugin SHARED src/rotatingSonarPlugin.cc)
target_link_libraries(rotating_sonar_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(vel src/vel.cc)
target_link_libraries(vel ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${roscpp_LIBRARIES})