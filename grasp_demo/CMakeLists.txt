cmake_minimum_required(VERSION 2.8.3)
project(grasp_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rostime std_msgs sensor_msgs message_filters cv_bridge image_transport 
compressed_image_transport tf compressed_depth_image_transport  geometry_msgs moveit_msgs moveit_ros_perception moveit_ros_planning_interface)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

add_service_files(
   FILES
   objtobaselink.srv
   cam2real.srv
   get_obj.srv
)

generate_messages(
        DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(

)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(CamtoReal src/CamtoReal.cpp)
target_link_libraries(CamtoReal
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(letter_grasp src/letter_grasp.cpp)
target_link_libraries(letter_grasp ${catkin_LIBRARIES})
add_dependencies(letter_grasp ${PROJECT_NAME}_generate_messages_cpp)

add_executable(yolo_obj src/yolo_obj.cpp)
target_link_libraries(yolo_obj ${catkin_LIBRARIES})
add_dependencies(yolo_obj ${PROJECT_NAME}_generate_messages_cpp)




