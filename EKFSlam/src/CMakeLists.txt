set( project_sources runSlamFromBag.cpp parseFilenameTexts.cpp EKFSlam.cpp ROSMessageManager.cpp)

#set(catkin_DIR "/opt/ros/melodic/share/catkin/cmake")
FIND_PACKAGE( catkin REQUIRED COMPONENTS cv_bridge rosbag roscpp sensor_msgs tf)
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE( Eigen3 3.3 REQUIRED NO_MODULE )
FIND_PACKAGE( PCL 1.3 REQUIRED COMPONENTS common io )
FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )


INCLUDE_DIRECTORIES( ${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )

LINK_DIRECTORIES( ${PCL_LIBRARY_DIRS} )

ADD_DEFINITIONS(${PCL_DEFINITIONS})

ADD_EXECUTABLE(${PROJECT_NAME} ${project_sources})
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} 
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_COMMON_LIBRARIES} 
    ${PCL_IO_LIBRARIES} 
    ${rosbag_LIBRARIES}
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ${HEADER_FILES})
#	 

	
#    
#    
    
set(EXECUTABLE_OUTPUT_PATH "../bin")
#add_custom_target(
#    run
#    COMMAND ${EXECUTABLE_OUTPUT_PATH}/RunSlamFromBagFile
#)
