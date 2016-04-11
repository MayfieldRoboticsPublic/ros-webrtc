cmake_minimum_required(VERSION 2.8.3)
project(ros_webrtc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  cv_bridge
  image_transport
  bondcpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED COMPONENTS system)
find_package(OpenCV)

## System dependencies also found via PkgConfig's conventions
pkg_check_modules(jingle REQUIRED libjingle)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Coverage
set(COVERAGE "OFF" CACHE STRING "Enable coverage generation.")
message(STATUS "Using COVERAGE: ${COVERAGE}")
if("${COVERAGE}" STREQUAL "ON")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Audio.msg
  Constraint.msg
  Data.msg
  DataChannel.msg
  MediaConstraints.msg
  Session.msg
  SessionEvent.msg
  SessionKey.msg
  Source.msg
  Track.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddSessionIceCandidate.srv
  BeginSession.srv
  ConnectSession.srv
  EndSession.srv
  GetHost.srv
  GetSession.srv
  SetSessionDescription.srv
  SendSessionData.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  ros_webrtc
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_webrtc
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
#)
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${jingle_INCLUDE_DIRS}
  ${jsoncpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(ros_webrtc
#   src/${PROJECT_NAME}/ros_webrtc.cpp
# )

## Declare a cpp executable
add_executable(host
   src/main.cpp
   src/config.cpp
   src/convert.cpp
   src/host.cpp
   src/media_constraints.cpp
   src/media_type.cpp
   src/renderer.cpp
   src/device_manager.cpp
   src/video_capture.cpp
   src/session.cpp
   src/util.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(host
  ros_webrtc_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(host
  ${catkin_LIBRARIES}
  ${jingle_STATIC_LDFLAGS}
  ${jsoncpp_STATIC_LDFLAGS}
  ${OpenCV_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

## Specify additional compile flags
foreach(cxx_flag "-std=c++0x" ${jingle_CFLAGS} ${jsoncpp_CFLAGS})
  set_property(TARGET host APPEND_STRING PROPERTY COMPILE_FLAGS " ${cxx_flag}")
endforeach(cxx_flag)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS host
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_webrtc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

if(CATKIN_ENABLE_TESTING)
  ## Data
  catkin_download_test_data(
    ${PROJECT_NAME}_sim_2015-02-23-15-14-55.bag
    https://s3-us-west-1.amazonaws.com/ai-mf-data/sim_2015-02-23-15-14-55.bag
    DESTINATION /tmp/${PROJECT_NAME}
    FILENAME sim_2015-02-23-15-14-55.bag
    MD5 1832f700f1066590f0323bceda474bd7
  )
  
  ## Integration
  find_package(rostest REQUIRED)
  foreach(T
      test/integration/host-test.launch
      test/integration/session-test.launch
    )
    add_rostest(${T})
  endforeach()
  
  ## Unit
  include_directories(src)
  catkin_add_gtest(unit_test
      test/unit/main.cpp
      src/media_constraints.cpp
      src/media_type.cpp
  )
  foreach(cxx_flag "-std=c++11" ${jingle_CFLAGS} ${jsoncpp_CFLAGS})
    set_property(TARGET unit_test APPEND_STRING PROPERTY COMPILE_FLAGS " ${cxx_flag}")
  endforeach(cxx_flag)
  target_link_libraries(unit_test
    ${catkin_LIBRARIES}
    ${jingle_STATIC_LDFLAGS}
    ${jsoncpp_STATIC_LDFLAGS}
    ${OpenCV_LIBRARIES}
    ${CMAKE_DL_LIBS}
  )
endif()
