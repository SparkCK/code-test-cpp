cmake_minimum_required(VERSION 3.20)
project(code-test)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0 -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3")

   
set(CMAKE_BUILD_TYPE Debug)
   
# find_package(NCURSES REQUIRED)
# pkg_check_modules(NCURSES REQUIRED ncurses)
find_package(PCL REQUIRED)
# find_package(Boost 1.78.0 EXACT REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/inlcude
    # ${Boost_INCLUDE_DIR}
    # ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}

)

add_subdirectory(src)
# execute_process(
#     COMMAND bash -c "pwd"
#     RESULT_VARIABLE result
#     OUTPUT_VARIABLE output
#     ERROR_VARIABLE error
# )
# message("result: ${result}")
# message("output: ${output}")
# message("error: ${error}")
