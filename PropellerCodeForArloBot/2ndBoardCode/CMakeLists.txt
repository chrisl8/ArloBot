cmake_minimum_required(VERSION 3.12)
find_package(PropWare REQUIRED)
set (BOARD QUICKSTART)
set (MODEL cmm)

include_directories(~/.arlobot)
include_directories(../ROSInterfaceForArloBot/include)

project(2ndBoardCode)
create_simple_executable(${PROJECT_NAME}
        2ndBoardCode.c
        )
target_link_libraries(${PROJECT_NAME} m)
