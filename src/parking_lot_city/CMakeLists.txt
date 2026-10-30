cmake_minimum_required(VERSION 3.8)
project(parking_lot_city NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models launch config rviz tools
        DESTINATION share/${PROJECT_NAME})

ament_package()
