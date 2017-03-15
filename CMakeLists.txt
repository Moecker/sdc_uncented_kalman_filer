cmake_minimum_required (VERSION 3.5)

project(UnscentedKalmanFilter LANGUAGES C CXX)

# Comppiler specific optons
if(MSVC)
    add_definitions("/MP -D_SCL_SECURE_NO_WARNINGS /W4 /WX /wd4005 /wd4127 /wd4481 /wd4482 /wd4505 /wd4512 /wd4714 /wd4800 /wd4189")
else()
    add_definitions(-std=c++0x)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Grab all files
file(GLOB sources src/*.cpp src/*.h)

add_executable(UnscentedKalmanFilter ${sources})
