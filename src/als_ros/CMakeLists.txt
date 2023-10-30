cmake_minimum_required(VERSION 3.0.2)
project(als_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES als
#  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversions
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(mcl src/mcl.cpp)
target_link_libraries(mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(sm src/sm.cpp)
target_link_libraries(sm ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(mrf_failure_detector src/mrf_failure_detector.cpp)
target_link_libraries(mrf_failure_detector ${catkin_LIBRARIES})

add_executable(gl_pose_sampler src/gl_pose_sampler.cpp)
target_link_libraries(gl_pose_sampler ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(classifier_dataset_generator src/classifier_dataset_generator.cpp)
target_link_libraries(classifier_dataset_generator ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(mae_classifier_learning src/mae_classifier_learning.cpp)
target_link_libraries(mae_classifier_learning ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(evaluator src/evaluator.cpp)
target_link_libraries(evaluator ${catkin_LIBRARIES})

add_executable(scan2pc src/scan2pc.cpp)
target_link_libraries(scan2pc ${catkin_LIBRARIES})

add_executable(slamer src/slamer.cpp)
target_link_libraries(slamer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PCL_LIBRARY_DIRS})
