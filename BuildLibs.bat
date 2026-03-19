@echo off
echo ===================================================
echo   Starting LibIGL Superbuild Process...
echo ===================================================
echo.

echo [1/3] Generating CMake build files...
cmake -B build
if %errorlevel% neq 0 (
    echo [ERROR] CMake configuration failed!
    pause
    exit /b %errorlevel%
)

echo.
echo [2/3] Compiling LibIGL (This will take a while)...
cmake --build build --config Release
if %errorlevel% neq 0 (
    echo [ERROR] Build failed! Check the logs above.
    pause
    exit /b %errorlevel%
)

echo.
echo [3/3] Cleaning up temporary and unnecessary files...

:: 1. Delete the massive temporary build folder
if exist build rmdir /s /q build

:: 2. Delete the CMake 'share' folder from your custom path
if exist Library\libigl\share rmdir /s /q Library\libigl\share

:: 3. Delete Eigen and Spectra from your custom path
if exist Library\libigl\include\Eigen rmdir /s /q Library\libigl\include\Eigen
if exist Library\libigl\include\Spectra rmdir /s /q Library\libigl\include\Spectra

echo.
echo ===================================================
echo   SUCCESS! 
echo   Your core libigl library is ready in Library\libigl
echo ===================================================
pause