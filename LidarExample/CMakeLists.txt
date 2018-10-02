cmake_minimum_required(VERSION 2.8)

project(LidarExample)

# Search for dependencies
find_package(HokuyoAIST)

# Set compiler flags
include_directories(${HokuyoAIST_INCLUDE_DIRS})
add_definitions(${HokuyoAIST_DEFINITIONS})

# Get laser data example
add_executable(${PROJECT_NAME}_main main.cpp)
target_link_libraries(${PROJECT_NAME}_main
    ${HokuyoAIST_LIBRARIES})

find_package(PythonLibs 2.7)
target_include_directories(${PROJECT_NAME}_main PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_main
        ${PYTHON_LIBRARIES})

# Get laser ID example
# add_executable(${PROJECT_NAME}_getid getid.cpp)
#target_link_libraries(${PROJECT_NAME}_getid
#    ${HokuyoAIST_LIBRARIES})
