cmake_minimum_required(VERSION 3.8)
project(ixblue_ins_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ixblue_ins_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)


## Specify libraries to link a library or executable target against
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

## System dependencies are found with CMake's conventions

# ixblue_stdbin_decoder must be set before Boost otherwise components are
# overridden (ixblue_stdbin_decoder only require system while we need
# program_options to build the packets replayer)
find_package(ixblue_stdbin_decoder REQUIRED)

find_package(Threads)

find_package(Boost REQUIRED COMPONENTS system)


###########
## Build ##
###########
include_directories(include)
add_library(${PROJECT_NAME}
        SHARED
        src/ip_listener.cpp
        src/udp_listener.cpp
        src/ros_publisher.cpp
        src/diagnostics_publisher.cpp
        )
target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}>"
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>
        )
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ixblue_stdbin_decoder::ixblue_stdbin_decoder)
ament_target_dependencies(
        ${PROJECT_NAME}
        rclcpp
        nav_msgs
        std_msgs
        sensor_msgs
        ixblue_ins_msgs
        diagnostic_msgs
        diagnostic_updater
        ixblue_stdbin_decoder
)


add_executable(
        ${PROJECT_NAME}_node
        src/main.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}>"
        $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>
        )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
target_link_libraries(${PROJECT_NAME}_node ${Boost_LIBRARIES} ${PROJECT_NAME} ixblue_stdbin_decoder::ixblue_stdbin_decoder)
ament_target_dependencies(
        ${PROJECT_NAME}_node
        rclcpp
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(DIRECTORY include/
        DESTINATION include)
install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
        )
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
        DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if (BUILD_TESTING)
    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_gmock REQUIRED)
    # PCAP and Boost program_options are used to build the packets replayer
    # that is only used for development so build and requise it as dependency
    # if tests are built
    find_package(PCAP REQUIRED)
    find_package(Boost REQUIRED COMPONENTS system program_options)

    add_executable(packets_replayer tools/packets_replayer.cpp)
    target_link_libraries(packets_replayer ${PCAP_LIBRARY} ${Boost_LIBRARIES} Threads::Threads)

    ## Add gtest based cpp test target and link libraries
    ament_add_gmock(${PROJECT_NAME}-test
            test/test_ixblue_ins_driver.cpp
            src/ros_publisher.cpp
            src/diagnostics_publisher.cpp
            )
    if (TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${Boost_LIBRARIES} ixblue_stdbin_decoder::ixblue_stdbin_decoder)
        ament_target_dependencies(${PROJECT_NAME}-test
                rclcpp
                ixblue_ins_msgs
                sensor_msgs
                diagnostic_msgs
                diagnostic_updater
                )
    endif ()
endif ()

###########################
## declare ament package ##
###########################
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        ixblue_stdbin_decoder
        ixblue_ins_msgs
)

ament_package(CONFIG_EXTRAS "cmake/ConfigExtras.cmake")
