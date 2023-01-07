cmake_minimum_required(VERSION 3.0.2)
project(Traj_tracking)

set(CMAKE_BUILD_TYPE "Release")

# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set(CMAKE_BUILD_TYPE "Debug")
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge
)

find_package(OpenCV REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Eigen3)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

# ################## source and driver ######################

add_executable(rc_ctl src/rc_ctl.cpp)
target_link_libraries(rc_ctl ${catkin_LIBRARIES} ${OpenCV_LIBS} dw)

add_executable(traj_ctl src/traj_ctl.cpp)
target_link_libraries(traj_ctl ${catkin_LIBRARIES} ${OpenCV_LIBS} dw)
