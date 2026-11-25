cmake_minimum_required(VERSION 3.20)
project(controlkit LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)

add_library(controlkit STATIC
  src/double_integrator.cpp
  src/integrators.cpp
  src/linearize.cpp
  src/lqr.cpp
  src/mpc.cpp
)
target_include_directories(controlkit PUBLIC include)
target_link_libraries(controlkit PUBLIC Eigen3::Eigen osqp::osqp)

add_executable(ball_demo apps/ball_demo.cpp)
target_link_libraries(ball_demo PRIVATE controlkit)

add_executable(ball_mpc_demo apps/ball_mpc_demo.cpp)
target_link_libraries(ball_mpc_demo PRIVATE controlkit)