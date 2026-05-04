@echo off
echo ===================================================
echo   Starting Eigen, LibIGL, and Instant Meshes Build
echo ===================================================
echo.

echo [1/4] Resolving External Dependencies (Eigen3)...
if not exist "Library\eigen\share\eigen3\cmake\Eigen3Config.cmake" (
    echo Downloading and configuring Eigen3...
    curl -s -L -o eigen.zip "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip"
    tar -xf eigen.zip
    cmake -S eigen-3.4.0 -B eigen_build -DCMAKE_INSTALL_PREFIX="%CD%\Library\eigen"
    cmake --install eigen_build
    rmdir /s /q eigen-3.4.0
    rmdir /s /q eigen_build
    del eigen.zip
)

echo.
echo [2/4] Generating and Compiling LibIGL...
cmake -B build -DCMAKE_CXX_STANDARD=17
cmake --build build --config Release

:: CLEANUP: Delete the LibIGL build artifacts after compilation
echo Cleaning up LibIGL build artifacts...
if exist "build" rmdir /s /q "build"

echo.
echo [3/4] Fetching and Surgically Patching Instant Meshes...
if not exist "Library\instant-meshes\CMakeLists.txt" (
    echo Downloading Instant Meshes repository...
    git clone --recursive https://github.com/wjakob/instant-meshes.git Library\instant-meshes
)

echo.
echo Detaching GUI, routing sub-modules, and forcing C++14 compatibility...

:: 1. Force wipe the build directory to ensure a clean slate
if exist "Library\instant-meshes\build" rmdir /s /q "Library\instant-meshes\build"

:: 2. Create the minimal CMakeLists.txt line-by-line to avoid Batch parser errors
set "NEW_CMAKE=Library\instant-meshes\CMakeLists.txt.new"
echo cmake_minimum_required(VERSION 3.10) > "%NEW_CMAKE%"
echo project(instant-meshes-core) >> "%NEW_CMAKE%"
echo. >> "%NEW_CMAKE%"
echo # CRITICAL: Revert to C++14. The bundled TBB relies on features removed in C++17. >> "%NEW_CMAKE%"
echo set(CMAKE_CXX_STANDARD 14) >> "%NEW_CMAKE%"
echo. >> "%NEW_CMAKE%"
echo # Fix Windows Math and Warnings >> "%NEW_CMAKE%"
echo if(MSVC) >> "%NEW_CMAKE%"
echo   add_definitions(-D_USE_MATH_DEFINES -D_CRT_SECURE_NO_WARNINGS -DM_PI=3.14159265358979323846) >> "%NEW_CMAKE%"
echo endif() >> "%NEW_CMAKE%"
echo. >> "%NEW_CMAKE%"
echo set(Eigen3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../eigen/share/eigen3/cmake") >> "%NEW_CMAKE%"
echo find_package(Eigen3 REQUIRED) >> "%NEW_CMAKE%"
echo. >> "%NEW_CMAKE%"
echo set(TBB_BUILD_TESTS OFF CACHE BOOL "" FORCE) >> "%NEW_CMAKE%"
echo add_subdirectory(ext/tbb) >> "%NEW_CMAKE%"
echo. >> "%NEW_CMAKE%"
echo # ROUTING: Add ALL internal dependency paths >> "%NEW_CMAKE%"
echo include_directories(include ext/eigen ext/tbb/include ext/dset ext/pss ext/rply ext/pcg32) >> "%NEW_CMAKE%"
echo. >> "%NEW_CMAKE%"
echo # Gather only core sources (Exclude Zombie GUI files) >> "%NEW_CMAKE%"
echo file(GLOB CORE_SRC "src/*.cpp") >> "%NEW_CMAKE%"
echo list(FILTER CORE_SRC EXCLUDE REGEX ".*(main|viewer|widgets|glutil|smoothcurve)\\.cpp$") >> "%NEW_CMAKE%"
echo. >> "%NEW_CMAKE%"
echo add_library(instant-meshes-core STATIC ${CORE_SRC}) >> "%NEW_CMAKE%"
echo target_link_libraries(instant-meshes-core PUBLIC Eigen3::Eigen tbb) >> "%NEW_CMAKE%"

:: Swap the files
move /y "%NEW_CMAKE%" Library\instant-meshes\CMakeLists.txt

echo.
echo [4/4] Compiling Instant Meshes Core...
cmake -S Library\instant-meshes -B Library\instant-meshes\build ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_PREFIX_PATH="%CD%\Library\eigen" ^
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
if %errorlevel% neq 0 goto error

cmake --build Library\instant-meshes\build --config Release
if %errorlevel% neq 0 goto error

echo.
echo ===================================================
echo   FINAL CLEANUP: Removing bloated .git folders...
echo ===================================================
:: Use PowerShell for a clean, recursive directory delete targeting hidden .git folders
powershell -Command "Get-ChildItem -Path 'Library' -Filter '.git' -Recurse -Force -Directory | Remove-Item -Recurse -Force"

echo.
echo ===================================================
echo   SUCCESS! All libraries built and cleaned!
echo ===================================================
pause
exit /b 0

:error
echo.
echo ===================================================
echo   [CRITICAL ERROR] A build step failed!
echo ===================================================
pause
exit /b %errorlevel%