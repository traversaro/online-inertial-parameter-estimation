CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME tutorials)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

## this is for testing only, please ignore if you are reading tutorials				
#INCLUDE(CTest)
### end test specific code

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")					

# Create everything needed to build our executable.
#ADD_EXECUTABLE(imageProc imageProc.cpp)
#ADD_EXECUTABLE(findLocation findLocation.cpp)
#ADD_EXECUTABLE(lookAtLocation lookAtLocation.cpp)
#ADD_EXECUTABLE(relay relay.cpp)
#ADD_EXECUTABLE(tutorial_arm tutorial_arm.cpp)
#ADD_EXECUTABLE(tutorial_arm_joint_impedance tutorial_arm_joint_impedance.cpp)
#ADD_EXECUTABLE(tutorial_rf_basic tutorial_rf_basic.cpp)
#ADD_EXECUTABLE(tutorial_rf_advanced tutorial_rf_advanced.cpp)
#ADD_EXECUTABLE(tutorial_rate_thread tutorial_rate_thread.cpp)
#ADD_EXECUTABLE(tutorial_module tutorial_module.cpp)
ADD_EXECUTABLE(cartesian_random_trajectory cartesian_random_trajectory.cpp)

TARGET_LINK_LIBRARIES(cartesian_random_trajectory ${YARP_LIBRARIES} icubmod)

