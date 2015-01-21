cmake_minimum_required(VERSION 2.8.3)
project(mo_tracker)

set(CMAKE_CXX_FLAGS "-std=c++11 -fomit-frame-pointer -fno-operator-names -march=native -mssse3 -msse2 -mfpmath=sse -fexcess-precision=fast -fno-strict-aliasing")

if(CMAKE_BUILD_TYPE)
  message("Build Type = ${CMAKE_BUILD_TYPE}")
else(CMAKE_BUILD_TYPE)
  message("Build Type not specified! Use command \"catkin_make -DCMAKE_BUILD_TYPE=Release\". ")
endif(CMAKE_BUILD_TYPE)

find_package(Boost REQUIRED COMPONENTS thread date_time filesystem program_options)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs message_generation tf)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Tracker.msg
  Trackers.msg
  Obstacle.msg
  Obstacles.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
        
if (DEFINED ENV{FLTKDIR})
  message("FLTKDIR = $ENV{FLTKDIR}")
  set(FLTKDIR $ENV{FLTKDIR})
else (DEFINED ENV{FLTKDIR})
  message("FLTKDIR is not specified. Using installed path.")
  set(FLTKDIR /usr/local/include)
endif (DEFINED ENV{FLTKDIR})

include_directories(include ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${FLTKDIR} )
link_directories(${FLTKDIR}/lib)
set(FLTK_LIBRARIES fltk2 Xi Xext X11 Xft Xinerama Xrender fontconfig)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES mo_tracker
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime tf
  DEPENDS ${FLTK_LIBRARIES}
)

add_library(mo_tracker STATIC
  src/CascadedCoords.cpp 
  src/CascadedVec.cpp
  src/ColorCout.cpp
  src/Coordinates.cpp 
  src/CvLRFViewer.cpp
#  src/LibSVM.cpp 
  src/LMS100Source.cpp 
  src/LRFSensor.cpp 
  src/LRFSource.cpp
  src/MatrixFuncs.cpp
  src/mmtimer.cpp
  src/MOTracker.cpp 
  src/MOTrackerFramework.cpp 
  src/MovingObject.cpp
  src/NHSensorFromFile.cpp 
  src/NHSensorManager.cpp 
  src/NHUtil.cpp
  src/observer.cpp 
  src/PolarVec.cpp 
  src/PolygonalRegion.cpp 
  src/SensorManagerGrabber.cpp
  src/SerialControlUnix.cpp 
  src/SocketUnix.cpp 
#  src/svm.cpp 
  src/TrackerConfig.cpp 
  src/TrackerLogger.cpp
  src/URGSource.cpp
)

add_executable(mo_tracker_node 
  src/FLTKPresenter.cpp 
  src/GridViewer.cpp
  src/JPDAEllipseFilter.cpp
  src/JPDAFilter.cpp 
  src/JPDARectangleFilter.cpp
  src/JPDATracker.cpp 
  src/main.cpp 
  src/ROSMessageSender.cpp
  src/ROSGrabber.cpp
  src/SimpleClassification.cpp  
  src/SJPDATrackerConfig.cpp  
#  src/SVMDataProcessing.cpp 
  src/TrackInitialization.cpp 
  src/TrackInitializationPassThrough.cpp
  src/TrackInitializationUsingGrid.cpp 
  src/TrackInitializationUsingPolarGrid.cpp
)
target_link_libraries(mo_tracker_node mo_tracker ${FLTK_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(mo_tracker_node mo_tracker mo_tracker_generate_messages_cpp)

add_executable(tracker_convertor src/tracker_convertor.cpp)
target_link_libraries(tracker_convertor ${catkin_LIBRARIES})
add_dependencies(tracker_convertor mo_tracker_generate_messages_cpp)
add_executable(obstacle_box_array_visualizer src/obstacle_box_array_visualizer.cpp)
target_link_libraries(obstacle_box_array_visualizer ${catkin_LIBRARIES})
add_dependencies(obstacle_box_array_visualizer mo_tracker_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS mo_tracker mo_tracker_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mo_tracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
