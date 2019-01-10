cmake_minimum_required(VERSION 2.8)

project(idyntree-ipopt-testing)


## Find packages
find_package(iDynTree REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} "main.cpp")

target_link_libraries(${PROJECT_NAME} PUBLIC
    iDynTree::idyntree-model
    iDynTree::idyntree-inverse-kinematics
    iDynTree::idyntree-high-level)


