# Olivier Stasse, Francois Keith, JRL, CNRS/AIST
# Creation: 4/11/2008
# History:
#
# Copyright CNRS/AIST

# For the test, we include the same files as the src
SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_CXXFLAGS}")

# For the test, we link with the library corresponding to the src,  which is not installed yet
#SET(${PROJECT_NAME}_test_LDFLAGS
#"${LIBDIR_KW}${${PROJECT_NAME}_BINARY_DIR}/src
#${LIBINCLUSION_KW}${PROJECT_NAME}-${PROJECT_VERSION}${LIBINCLUSION_EXT}
#${${PROJECT_NAME}_src_LDFLAGS}")

SET(${PROJECT_NAME}_test_LDFLAGS 
"${LIBDIR_KW}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBINCLUSION_KW}${PROJECT_NAME}-${PROJECT_VERSION}${LIBINCLUSION_EXT} ${${PROJECT_NAME}_src_LDFLAGS}")

MESSAGE(STATUS " unitTesting: ${UNITTESTING_CXXFLAGS}")

#ADD_EXECUTABLE(name_of_the_exectuable source_test.cpp)
#SET_TARGET_PROPERTIES(name_of_the_exectuable
#	              PROPERTIES
#		      COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
#		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
#)
#ADD_DEPENDENCIES (name_of_the_exectuable ${PROJECT_NAME})

ADD_EXECUTABLE( npss oneStepTest2.cpp)
SET_TARGET_PROPERTIES( npss
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
          	      LINK_FLAGS "-lqhull -lgfortran ${${PROJECT_NAME}_test_LDFLAGS} -Wl,-R${PROJECT_SOURCE_DIR}/build/src ${PROJECT_SOURCE_DIR}/build/src/lib${PROJECT_NAME}.so"

)

ADD_EXECUTABLE( testscd testSCD.cpp)
SET_TARGET_PROPERTIES( testscd
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
          	      LINK_FLAGS "-lqhull -lgfortran ${${PROJECT_NAME}_test_LDFLAGS} -Wl,-R${PROJECT_SOURCE_DIR}/build/src ${PROJECT_SOURCE_DIR}/build/src/lib${PROJECT_NAME}.so"

)

ADD_EXECUTABLE( oss oneStepTest.cpp)
SET_TARGET_PROPERTIES( oss
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
          	      LINK_FLAGS "-lqhull -lgfortran ${${PROJECT_NAME}_test_LDFLAGS} -Wl,-R${PROJECT_SOURCE_DIR}/build/src ${PROJECT_SOURCE_DIR}/build/src/lib${PROJECT_NAME}.so"

)
#ADD_DEPENDENCIES (oss ${PROJECT_NAME})
ADD_DEPENDENCIES (npss ${PROJECT_NAME})
ADD_DEPENDENCIES (oss ${PROJECT_NAME})
