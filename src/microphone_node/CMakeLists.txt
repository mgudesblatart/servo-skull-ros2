cmake_minimum_required(VERSION 3.8)
project(microphone_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(servo_skull_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

install(
  DIRECTORY include/
  DESTINATION include
)

include_directories(
  src
  ${PORTAUDIO_INCLUDE_DIRS}
  src/spl_sqrt_floor
  src/rtc_base
  src/rtc_base/numerics
  src/rtc_base/system
  src/signal_processing
  src/signal_processing/include
  src/webrtc_vad
  include
)

# Add WebRTC VAD and dependencies sources
file(GLOB SIGNAL_PROCESSING_SRC
  src/signal_processing/*.c
  src/signal_processing/*.cc
)
file(GLOB RTC_BASE_SRC
  src/rtc_base/*.cc
)
file(GLOB SPL_SQRT_FLOOR_SRC
  src/spl_sqrt_floor/*.c
)
file(GLOB WEBRTC_VAD_C_SRC
  src/webrtc_vad/*.c
  )
set(WEBRTC_VAD_SRC
  ${WEBRTC_VAD_C_SRC}
  ${SIGNAL_PROCESSING_SRC}
  ${RTC_BASE_SRC}
  ${SPL_SQRT_FLOOR_SRC}
)
add_executable(microphone_node src/microphone_node.cpp)
target_sources(microphone_node PRIVATE ${WEBRTC_VAD_SRC})
target_link_libraries(microphone_node ${PORTAUDIO_LIBRARIES})
ament_target_dependencies(microphone_node rclcpp servo_skull_msgs std_msgs)

install(TARGETS microphone_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_webrtc_vad_wrapper test/test_webrtc_vad_wrapper.cpp)
  target_sources(test_webrtc_vad_wrapper PRIVATE ${WEBRTC_VAD_SRC})
  target_link_libraries(test_webrtc_vad_wrapper pthread)
  ament_add_gtest(test_audio_queue test/test_audio_queue.cpp)
  target_link_libraries(test_audio_queue pthread)
endif()

ament_package()
