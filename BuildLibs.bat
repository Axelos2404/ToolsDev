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
    
    :: Configure and install Eigen locally
    cmake -S eigen-3.4.0 -B eigen_build -DCMAKE_INSTALL_PREFIX="%CD%\Library\eigen"
    if %errorlevel% neq 0 goto error
    
    cmake --install eigen_build
    if %errorlevel% neq 0 goto error
    
    :: Clean up
    rmdir /s /q eigen-3.4.0
    rmdir /s /q eigen_build
    del eigen.zip
) else (
    echo Eigen3 already installed. Skipping download.
)

echo.
echo [2/4] Generating and Compiling LibIGL...
cmake -B build -DCMAKE_CXX_STANDARD=17
if %errorlevel% neq 0 goto error

cmake --build build --config Release
if %errorlevel% neq 0 goto error

echo.
echo [3/4] Fetching Instant Meshes...
if not exist "Library\instant-meshes\CMakeLists.txt" (
    echo Downloading Instant Meshes repository...
    if exist "Library\instant-meshes" rmdir /s /q "Library\instant-meshes"
    :: We MUST use --recursive to grab its TBB and PCG32 submodules!
    git clone --recursive https://github.com/wjakob/instant-meshes.git Library\instant-meshes
)

echo.
echo [4/4] Compiling Instant Meshes Core...
cmake -S Library\instant-meshes -B Library\instant-meshes\build ^
  -DCMAKE_CXX_STANDARD=17 ^
  -DCMAKE_BUILD_TYPE=Release
if %errorlevel% neq 0 goto error

cmake --build Library\instant-meshes\build --config Release
if %errorlevel% neq 0 goto error

echo.
echo ===================================================
echo   SUCCESS! 
echo   The environment is fully built and ready!
echo ===================================================
pause
exit /b 0

:: --- ERROR HANDLER ---
:error
echo.
echo ===================================================
echo   [CRITICAL ERROR] A build step failed!
echo   Please scroll up to read the CMake error message.
echo ===================================================
pause
exit /b %errorlevel%