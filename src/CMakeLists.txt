# set(EXEC_TARGET_SOURCES)
# file(GLOB EXEC_TARGET_SOURCES *.cpp)

link_directories(
    ${PCL_LIBRARY_DIRS}
)


add_executable(${PROJECT_NAME} 
    # ${EXEC_TARGET_SOURCES}
    # main.cpp
    main_pcl_visualizer.cpp
    # obstacle_visualizer.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD "17"
    CXX_STANDARD_REQUIRED "YES"
    CXX_EXTENSIONS "NO"
    ARCHIVE_OUTPUT_DIRECTORY "${TARGET_OUT_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${TARGET_OUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${TARGET_OUT_DIR}"
)

target_link_libraries(${PROJECT_NAME}
    # boost_json
    # boost_filesystem
    # ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    # ${NCURSES_LIBRARIES}
)

INSTALL(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/bin
)