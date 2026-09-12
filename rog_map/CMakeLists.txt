cmake_minimum_required(VERSION 3.10)
project(rog_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

add_definitions(-DORIGIN_AT_CORNER)
#add_definitions(-DORIGIN_AT_CENTER)



find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        visualization_msgs
        message_filters
        dynamic_reconfigure
        tf2_ros
        tf2_eigen
        rog_map_msgs
        diagnostic_msgs
        std_srvs
        )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

#add dynamic reconfigure api
set (DYNAMIC_RECONFIGURE_PATH config/visualization.cfg)

generate_dynamic_reconfigure_options(
        ${DYNAMIC_RECONFIGURE_PATH}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rog_map
        CATKIN_DEPENDS roscpp std_msgs dynamic_reconfigure tf2_ros rog_map_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp include/*.cpp)
add_library(rog_map STATIC ${srcs}
        )
target_include_directories(rog_map PUBLIC include)
target_link_libraries(rog_map PUBLIC
        ${THIRD_PARTY})
add_dependencies(rog_map ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(rog_map_node apps/rog_map_node.cpp)
target_link_libraries(rog_map_node rog_map ${catkin_LIBRARIES})
add_dependencies(rog_map_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
install(TARGETS rog_map rog_map_node ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/local_map_node.test)
  add_rostest_gtest(local_mapper_test test/local_mapper.test test/local_mapper_test.cpp)
  if(TARGET local_mapper_test)
    target_link_libraries(local_mapper_test rog_map ${catkin_LIBRARIES})
    add_dependencies(local_mapper_test ${catkin_EXPORTED_TARGETS})
  endif()
endif()

catkin_install_python(PROGRAMS scripts/measure_local_map.py test/local_map_node_test.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
