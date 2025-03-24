cmake_minimum_required(VERSION 3.1...3.14)
project(audio_tools C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(SDL2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AudioData.msg
  msg/AudioDataStamped.msg
  msg/AudioInfo.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(audio_capture_node src/audio_capture_node.cpp)

add_compile_options(audio_capture_node -Wall)

ament_target_dependencies(audio_capture_node
    rclcpp
    std_msgs
)

target_link_libraries(audio_capture_node "${cpp_typesupport_target}" ${SDL2_LIBRARIES}) 
target_include_directories(audio_capture_node PUBLIC ${SDL2_INCLUDE_DIRS})

install(TARGETS
audio_capture_node
DESTINATION lib/${PROJECT_NAME}) 


add_executable(audio_playback_node src/audio_playback_node.cpp)
target_link_libraries(audio_playback_node "${cpp_typesupport_target}" ${SDL2_LIBRARIES}) 
target_include_directories(audio_playback_node PUBLIC ${SDL2_INCLUDE_DIRS})
add_compile_options(audio_playback_node -Wall)

ament_target_dependencies(audio_playback_node
    rclcpp
    std_msgs
)


install(TARGETS
audio_playback_node
DESTINATION lib/${PROJECT_NAME}) 
ament_package()