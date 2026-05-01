@echo off
echo ===================================================
echo   Starting LibIGL & CoMISo Superbuild Process...
echo ===================================================
echo.

echo [1/5] Resolving External Dependencies (GMM++)...
:: Download GMM++ BEFORE CMake runs so CoMISo natively detects it
if not exist "Library\comiso\include\gmm" (
    echo Downloading missing GMM++ headers...
    curl -s -L -o getfem.zip "https://github.com/getfem-doc/getfem/archive/refs/heads/master.zip"
    tar -xf getfem.zip getfem-master/src/gmm
    xcopy /E /I /Y /Q "getfem-master\src\gmm" "Library\comiso\include\gmm"
    rmdir /s /q getfem-master
    del getfem.zip
    
    :: Create the safe config file
    echo // Empty safe config > "Library\comiso\include\gmm\gmm_arch_config.h"
) else (
    echo GMM++ already installed. Skipping download.
)

echo.
echo [2/5] Generating CMake build files...
cmake -B build
if %errorlevel% neq 0 (
    echo [ERROR] CMake configuration failed!
    pause
    exit /b %errorlevel%
)

echo.
echo [3/5] Compiling Libraries (This will take a while)...
cmake --build build --config Release
if %errorlevel% neq 0 (
    echo [ERROR] Build failed! Check the logs above.
    pause
    exit /b %errorlevel%
)

echo.
echo [4/5] Cleaning up massive temporary files...
if exist build rmdir /s /q build
if exist Library\libigl\share rmdir /s /q Library\libigl\share
if exist Library\libigl\lib\cmake rmdir /s /q Library\libigl\lib\cmake
if exist Library\libigl\include\Eigen rmdir /s /q Library\libigl\include\Eigen
if exist Library\libigl\include\Spectra rmdir /s /q Library\libigl\include\Spectra

echo.
echo [5/5] Applying Custom Auto-Retopology Patches...

:: Patch A: Fetch the CoMISo bridge from your GitHub branch
if not exist "Library\libigl\include\igl\copyleft\comiso" mkdir "Library\libigl\include\igl\copyleft\comiso"
echo Downloading comiso branch...
git clone -b copyleft/comiso --single-branch https://github.com/Axelos2404/ToolsDev.git temp_comiso_repo
xcopy /E /I /Y /Q "temp_comiso_repo\comiso" "Library\libigl\include\igl\copyleft\comiso"
rmdir /s /q temp_comiso_repo

:: Patch B: Automate the missing principal_curvature.cpp download
if not exist "Library\libigl\include\igl\principal_curvature.cpp" (
    echo Downloading missing principal_curvature.cpp...
    curl -s -o "Library\libigl\include\igl\principal_curvature.cpp" -L "https://raw.githubusercontent.com/libigl/libigl/v2.6.0/include/igl/principal_curvature.cpp"
)

echo.
echo ===================================================
echo   SUCCESS! 
echo   The environment is fully built, patched, and ready!
echo ===================================================
pause