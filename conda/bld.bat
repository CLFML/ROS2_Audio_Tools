rmdir /S /Q build
rmdir /S /Q install

set PATH=%BUILD_PREFIX%\Scripts;%PATH%

:: Then the build
SET CMAKE_BUILD_TYPE=Release
colcon build ^
  --merge-install ^
  --install-base=%CONDA_PREFIX% ^
  --parallel-workers=%NUMBER_OF_PROCESSORS% ^
  --cmake-args -DCMAKE_BUILD_TYPE=Release ^
  --event-handlers console_direct+ 
