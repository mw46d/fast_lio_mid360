cmake_minimum_required(VERSION 3.14)
project(fast_lio)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DBUILDING_ROS2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(eigen_conversions REQUIRED)

# set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.12 REQUIRED)
set(PCL_INCLUDE_DIRS /usr/include/pcl-1.12)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

# generate custom msg headers
set(FLIO_INTERFACES fast_lio_interfaces)
rosidl_generate_interfaces(${FLIO_INTERFACES}
  "msg/Pose6D.msg"
  DEPENDENCIES builtin_interfaces std_msgs std_srvs geometry_msgs
  LIBRARY_NAME ${PROJECT_NAME}
  )

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)

# get include directories of custom msg headers
if(HUMBLE_ROS STREQUAL "humble")
  rosidl_get_typesupport_target(cpp_typesupport_target
  ${FLIO_INTERFACES} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping "${cpp_typesupport_target}")
else()
  set(FLIO_INTERFACE_TARGET "${FLIO_INTERFACES}__rosidl_typesupport_cpp")
  add_dependencies(${PROJECT_NAME} ${FLIO_INTERFACES})
  get_target_property(FLIO_INTERFACES_INCLUDE_DIRECTORIES ${FLIO_INTERFACE_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
endif()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${FLIO_INTERFACES_INCLUDE_DIRECTORIES}
  include)

ament_target_dependencies(fastlio_mapping rclcpp cv_bridge geometry_msgs image_transport livox_ros_driver2 nav_msgs pcl_ros sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs visualization_msgs)
target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBRARIES})
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

# ament_auto_package(INSTALL_TO_SHARE
#   config
#   launch
# )
ament_package()
