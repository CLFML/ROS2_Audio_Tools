:: Get release version from GitHub, strip leading 'v'
FOR /F "delims=" %%A IN ('gh release view --json tagName --jq ".tagName"') DO (
    SET "RELEASE_VERSION=%%A"
)
:: Remove leading 'v'
SET "RELEASE_VERSION=%RELEASE_VERSION:v=%"

:: Create isolated build workspace
mkdir "%TEMP%\ros-audio-build\src"

:: Use robocopy to copy files except .pixi, pixi.toml, and .git
robocopy . "%TEMP%\ros-audio-build\src" /E /XD .pixi .git /XF pixi.toml

:: Copy conda recipe separately
robocopy conda "%TEMP%\ros-audio-build\conda" /E

:: Set SRC_PATH environment variable
SET "SRC_PATH=%TEMP%\ros-audio-build\src"

:: Add conda-build and conda-verify to the environment using pixi
pixi add conda-build conda-verify

:: Run the build
pixi run conda build . ^
    -c https://prefix.dev/robostack-jazzy ^
    -c conda-forge
