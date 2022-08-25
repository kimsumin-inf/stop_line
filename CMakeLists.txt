cmake_minimum_required(VERSION 3.0.2)
project(stop_line)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS cv_bridge data_transfer_msg roscpp sensor_msgs std_msgs)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/stop_line_node.cpp src/stop_line.cpp include/stop_line/stop_line.h include/stop_line/stop_line.h src/stop_line.cpp
        )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})