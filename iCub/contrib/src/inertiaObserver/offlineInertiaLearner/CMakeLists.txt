CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME offlineInertiaLearner)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

INCLUDE_DIRECTORIES(${learningMachine_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

## this is for testing only, please ignore if you are reading tutorials				
#INCLUDE(CTest)
### end test specific code

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")					

ADD_EXECUTABLE(offlineInertiaLearner offlineInertiaLearner.cpp MatVetIO.cpp)

TARGET_LINK_LIBRARIES(offlineInertiaLearner learningMachine ${YARP_LIBRARIES})

