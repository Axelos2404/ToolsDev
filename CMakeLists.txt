cmake_minimum_required(VERSION 3.14)
project(LibraryFetcher)

include(ExternalProject)

# ------------------------------------------------------------------------------
# 1. Define your target directory
# ------------------------------------------------------------------------------
set(MY_VS_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Library/libigl" CACHE PATH "Where to put the libs")

message(STATUS "Libraries will be installed to: ${MY_VS_INSTALL_DIR}")

# ------------------------------------------------------------------------------
# 2. Configure the LibIGL download and build instructions
# ------------------------------------------------------------------------------
ExternalProject_Add(
    libigl_build
    
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG        v2.6.0 # Matching your current log
    
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${MY_VS_INSTALL_DIR} 
        -DLIBIGL_USE_STATIC_LIBRARY=ON              
        -DCMAKE_BUILD_TYPE=Release                  
        
        # Turn off the heavy stuff
        -DLIBIGL_BUILD_TESTS=OFF
        -DLIBIGL_BUILD_TUTORIALS=OFF
        -DLIBIGL_EMBREE=OFF                    
        -DLIBIGL_COPYLEFT_CGAL=OFF
        
        # Since I'm using Qt, I don't need libigl's built-in window viewers
        -DLIBIGL_OPENGL=OFF
        -DLIBIGL_GLFW=OFF
        -DLIBIGL_IMGUI=OFF
)