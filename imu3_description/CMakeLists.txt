cmake_minimum_required(VERSION 2.8.3)
project(imu3_description)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  urdf
)


include_directories(include ${catkin_INCLUDE_DIRS})




catkin_package(
  CATKIN_DEPENDS 
)


include_directories(${catkin_INCLUDE_DIRS} include $ENV{HOME}/install/include)



# Declare a C++ executable
# add_executable(imu3_node src/main.cpp src/IMUGL.cpp )
## Add cmake target dependencies of the executable
## same as for the library above 
# add_dependencies(imu3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
# Specify libraries to link a library or executable target against
# target_link_libraries(imu3_node ${catkin_LIBRARIES} $ENV{HOME}/install/lib nlopt m nlopt_cxx )
# target_link_libraries(imu3_node ${catkin_LIBRARIES}  nlopt  m)

# Declare a C++ executable
# add_executable(imu3_calibration_node src/mag_calibration/main_mag_calibration.cpp  src/mag_calibration/IMUGL_mag_calibration.cpp)
# ## Add cmake target dependencies of the executable same as for the library above
# add_dependencies(imu3_calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
# # Specify libraries to link a library or executable target against
# target_link_libraries(imu3_calibration_node ${catkin_LIBRARIES} )