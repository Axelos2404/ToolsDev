@echo off
echo ===================================================
echo   Starting LibIGL, CoMISo, and QEx Superbuild...
echo ===================================================
echo.

echo [1/8] Fetching CoMISo Source Code...
if not exist "Library\comiso\CMakeLists.txt" (
    echo Downloading official CoMISo repository...
    if exist "Library\comiso" rmdir /s /q "Library\comiso"
    git clone https://github.com/libigl/CoMISo.git Library\comiso
)

echo.
echo [2/8] Resolving External Dependencies (Eigen3)...
if not exist "Library\Eigen\share\eigen3\cmake\Eigen3Config.cmake" (
    echo Downloading and configuring Eigen3...
    curl -s -L -o eigen.zip "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip"
    tar -xf eigen.zip
    
    :: Configure and install Eigen locally
    cmake -S eigen-3.4.0 -B eigen_build -DCMAKE_INSTALL_PREFIX="%CD%\Library\Eigen"
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
echo [3/8] Resolving External Dependencies (GMM++)...
if not exist "Library\comiso\include\gmm" (
    echo Downloading missing GMM++ headers...
    curl -s -L -o getfem.zip "https://github.com/getfem-doc/getfem/archive/refs/heads/master.zip"
    tar -xf getfem.zip getfem-master/src/gmm
    xcopy /E /I /Y /Q "getfem-master\src\gmm" "Library\comiso\include\gmm"
    rmdir /s /q getfem-master
    del getfem.zip
    
    :: Create the safe config file to bypass CMake variables
    echo // Empty safe config > "Library\comiso\include\gmm\gmm_arch_config.h"
)

echo.
echo [4/8] Compiling CoMISo with GMM++ and Eigen3...
:: Inject Eigen3 search directly into the top of CoMISo's CMakeLists so it can compile standalone!
if not exist "Library\comiso\patched_eigen.flag" (
    echo find_package^(Eigen3 REQUIRED CONFIG PATHS "%CD:\=/%/Library/Eigen/share/eigen3/cmake"^) > Library\comiso\new_cmake.txt
    type Library\comiso\CMakeLists.txt >> Library\comiso\new_cmake.txt
    move /Y Library\comiso\new_cmake.txt Library\comiso\CMakeLists.txt
    echo done > Library\comiso\patched_eigen.flag
)

:: Force C++17, force GMM inclusion, and disable warnings
cmake -S Library\comiso -B Library\comiso\build ^
  -DCMAKE_CXX_STANDARD=17 ^
  -DCOMISO_GMM_AVAILABLE=ON ^
  -DCMAKE_CXX_FLAGS="/I\"%CD%\Library\comiso\include\" /D_CRT_SECURE_NO_DEPRECATE /D_SCL_SECURE_NO_DEPRECATE /EHsc /std:c++17 /DCOMISO_GMM_AVAILABLE=1"
if %errorlevel% neq 0 goto error

cmake --build Library\comiso\build --config Release
if %errorlevel% neq 0 goto error

:: Organize the output so Visual Studio can find it easily
if not exist "Library\comiso\lib" mkdir "Library\comiso\lib"
copy /Y "Library\comiso\build\Release\CoMISo.lib" "Library\comiso\lib\CoMISo.lib"

echo.
echo [5/8] Generating LibIGL CMake build files...
cmake -B build -DCMAKE_CXX_STANDARD=17
if %errorlevel% neq 0 goto error

echo.
echo [6/8] Compiling LibIGL...
cmake --build build --config Release
if %errorlevel% neq 0 goto error

echo.
echo [7/8] Fetching and Compiling libQEx (Quad Extraction)...
if not exist "Library\qex\CMakeLists.txt" (
    echo Downloading official libQEx repository...
    if exist "Library\qex" rmdir /s /q "Library\qex"
    git clone https://github.com/hcebke/libQEx.git Library\qex
)

:: Configure and build libQEx, injecting the path to our local Eigen installation
cmake -S Library\qex -B Library\qex\build ^
  -DCMAKE_CXX_STANDARD=17 ^
  -DEigen3_DIR="%CD%\Library\Eigen\share\eigen3\cmake" ^
  -DCMAKE_CXX_FLAGS="/I\"%CD%\Library\Eigen\include\eigen3\" /EHsc /D_CRT_SECURE_NO_DEPRECATE /D_SCL_SECURE_NO_DEPRECATE"
if %errorlevel% neq 0 goto error

cmake --build Library\qex\build --config Release
if %errorlevel% neq 0 goto error

:: Organize the output so Visual Studio can find it easily
if not exist "Library\qex\lib" mkdir "Library\qex\lib"
copy /Y "Library\qex\build\Release\*.lib" "Library\qex\lib\"

echo.
echo [8/8] Applying Custom Auto-Retopology Patches...

:: Patch A: Fetch the CoMISo bridge from my GitHub branch
if not exist "Library\libigl\include\igl\copyleft\comiso" mkdir "Library\libigl\include\igl\copyleft\comiso"
echo Downloading comiso branch...
git clone -b copyleft/comiso --single-branch https://github.com/Axelos2404/ToolsDev.git temp_comiso_repo
xcopy /E /I /Y /Q "temp_comiso_repo\comiso" "Library\libigl\include\igl\copyleft\comiso"
rmdir /s /q temp_comiso_repo

:: Patch B: Automate missing .cpp downloads for strict template instantiations
if not exist "Library\libigl\include\igl\principal_curvature.cpp" (
    echo Downloading missing principal_curvature.cpp...
    curl -s -o "Library\libigl\include\igl\principal_curvature.cpp" -L "https://raw.githubusercontent.com/libigl/libigl/v2.6.0/include/igl/principal_curvature.cpp"
)

if not exist "Library\libigl\include\igl\local_basis.cpp" (
    echo Downloading missing local_basis.cpp...
    curl -s -o "Library\libigl\include\igl\local_basis.cpp" -L "https://raw.githubusercontent.com/libigl/libigl/v2.6.0/include/igl/local_basis.cpp"
)

echo.
echo ===================================================
echo   Cleaning up massive temporary files...
if exist build rmdir /s /q build
if exist Library\comiso\build rmdir /s /q Library\comiso\build
if exist Library\qex\build rmdir /s /q Library\qex\build
if exist Library\libigl\share rmdir /s /q Library\libigl\share
if exist Library\libigl\lib\cmake rmdir /s /q Library\libigl\lib\cmake
if exist Library\libigl\include\Eigen rmdir /s /q Library\libigl\include\Eigen
if exist Library\libigl\include\Spectra rmdir /s /q Library\libigl\include\Spectra

:: Remove source code bloat to save hard drive space
if exist "Library\comiso\Examples" rmdir /s /q "Library\comiso\Examples"
if exist "Library\comiso\QtWidgets" rmdir /s /q "Library\comiso\QtWidgets"
if exist "Library\comiso\.git" rmdir /s /q "Library\comiso\.git"
if exist "Library\comiso\cmake" rmdir /s /q "Library\comiso\cmake"
if exist "Library\qex\.git" rmdir /s /q "Library\qex\.git"

echo.
echo   SUCCESS! 
echo   The environment is fully built, patched, and ready!
echo ===================================================
pause
exit /b 0

:: --- ERROR HANDLER ---
:error
echo.
echo ===================================================
echo   [CRITICAL ERROR] A build step failed!
echo   Please read the error message above to see why.
echo ===================================================
pause
exit /b %errorlevel%