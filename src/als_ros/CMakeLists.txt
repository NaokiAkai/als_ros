cmake_minimum_required(VERSION 3.0.2)
project(als_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES als
#  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversions
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(mcl src/mcl.cpp)
target_link_libraries(mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

add_executable(mrf_failure_detector src/mrf_failure_detector.cpp)
target_link_libraries(mrf_failure_detector ${catkin_LIBRARIES})

add_executable(gl_pose_sampler src/gl_pose_sampler.cpp)
target_link_libraries(gl_pose_sampler ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(classifier_dataset_generator src/classifier_dataset_generator.cpp)
target_link_libraries(classifier_dataset_generator ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(mae_classifier_learning src/mae_classifier_learning.cpp)
target_link_libraries(mae_classifier_learning ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_executable(evaluator src/evaluator.cpp)
target_link_libraries(evaluator ${catkin_LIBRARIES})

add_executable(scan2pc src/scan2pc.cpp)
target_link_libraries(scan2pc ${catkin_LIBRARIES})

