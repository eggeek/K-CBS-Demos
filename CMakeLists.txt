cmake_minimum_required (VERSION 3.16)
project(K-CBS-Examples)

set (CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "-I${CMAKE_SOURCE_DIR}/ompl/src -I${CMAKE_SOURCE_DIR}/ompl/build/src -I/usr/include/eigen3")
set(DEV_CXX_FLAGS "-g -ggdb -O0 -fno-omit-frame-pointer")
set(RELEASE_CXX_FLAGS "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEV_CXX_FLAGS}")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../bin)
set(OMPL_LIBRARIES "${CMAKE_SOURCE_DIR}/ompl/build/lib/libompl.so")


find_package(ompl REQUIRED) # make sure this is the ompl with the K-CBS implementation

include_directories(
    includes
)

# file(GLOB DEMO_SOURCES RELATIVE ${CMAKE_SOURCE_DIR}/src/ src/*.cpp )
set(DEMO_SOURCES 
  expr.cpp
  # simple_expr.cpp
)

set(ALL_SOURCES 
  ./includes/customizedKCBS.cpp
)


include(CMakePrintHelpers)
cmake_print_variables(DEMO_SOURCES)
cmake_print_variables(CMAKE_SOURCE_DIR)

foreach( demoFile ${DEMO_SOURCES} )
    string( REPLACE ".cpp" "" demoName ${demoFile} )
    cmake_print_variables(demoName)
    add_executable( ${demoName} src/${demoFile} ${ALL_SOURCES})
    # Make sure YourLib is linked to each app
    target_link_libraries( ${demoName} ${OMPL_LIBRARIES} )
endforeach( demoFile ${DEMO_SOURCES} )

# ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")
