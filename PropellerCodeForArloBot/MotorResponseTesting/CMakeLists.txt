cmake_minimum_required(VERSION 3.12)
find_package(PropWare REQUIRED)
set (BOARD ACTIVITYBOARD)
set (MODEL cmm)

include_directories(~/.arlobot)
include_directories(../include)

project(MotorResponseTesting)
create_simple_executable(${PROJECT_NAME}
        MotorResponseTesting.c
        )
target_link_libraries(${PROJECT_NAME} m)
